#!/usr/bin/env python

# Copyright 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for utils.py's node dependency-injection behavior.

get_controller_managers(), ControllerManagerLister and ControllerLister used
to each create their own private rclpy node. They now take a node supplied by
the caller (e.g. rqt's shared context.node) and never spin it themselves.
These tests exercise that contract -- including ControllerLister's owned
service client lifetime (close()) and its context-shutdown-aware wait -- with
minimal fake node/client/future stubs, so no live ROS node, context, or
executor is involved. In particular, none of these tests call rclpy.init(),
so any regression that reintroduces private node creation would fail here
immediately.

Run with:
    pytest rqt_joint_trajectory_controller/test/test_utils.py -v
"""

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rqt_joint_trajectory_controller import utils

# ---------------------------------------------------------------------------
# Fake node/client/future stubs
# ---------------------------------------------------------------------------


class _FakeGraphNode:
    """Minimal node stub for get_controller_managers()/ControllerManagerLister.

    Only get_service_names_and_types() is needed -- these never spin or call
    services, they only do synchronous ROS graph introspection.
    """

    def __init__(self, services):
        self._services = services

    def get_service_names_and_types(self):
        return self._services


def _cm_services_under(ns):
    """Build a get_service_names_and_types()-style list exposing all controller
    manager services under namespace ns, so is_controller_manager(node, ns) is True.
    """
    prefix = "" if ns in ("", "/") else ns
    return [(f"{prefix}/{name}", [srv_type]) for name, srv_type in utils.cm_services.items()]


class _FakeContext:
    """Minimal stand-in for rclpy.context.Context: only ok() is needed."""

    def __init__(self, ok=True):
        self._ok = ok

    def ok(self):
        return self._ok


class _FakeFuture:
    """
    Minimal stand-in for rclpy.task.Future, driven manually by tests.

    Mirrors the two behaviors ControllerLister.__call__() relies on: done
    callbacks fire immediately if added after completion/cancellation
    (real rclpy Future does this too, to avoid missing an already-done
    future), and cancel() is a no-op once the future is already done.
    """

    def __init__(self):
        self._callbacks = []
        self._result = None
        self._done = False
        self._cancelled = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done or self._cancelled:
            callback(self)

    def result(self):
        return self._result

    def cancelled(self):
        return self._cancelled

    def cancel(self):
        if self._done or self._cancelled:
            return
        self._cancelled = True
        for callback in self._callbacks:
            callback(self)

    def complete(self, result):
        self._result = result
        self._done = True
        for callback in self._callbacks:
            callback(self)


class _FakeListControllersClient:
    """
    Mirrors rclpy.client.Client's pending-request bookkeeping closely enough
    to assert on it: call_async() tracks the future in pending_requests until
    it is done or cancelled, exactly like Client.remove_pending_request()
    being wired up as the future's own done-callback in real rclpy.
    """

    def __init__(self):
        self.requests = []
        self.last_future = None
        self.pending_requests = {}

    def call_async(self, request):
        self.requests.append(request)
        future = _FakeFuture()
        self.last_future = future
        seq = len(self.requests)
        self.pending_requests[seq] = future
        future.add_done_callback(lambda _future, seq=seq: self.pending_requests.pop(seq, None))
        return future


class _FakeControllerListerNode:
    """Minimal node stub for ControllerLister: create_client()/destroy_client()/context."""

    def __init__(self, context=None):
        self.context = context if context is not None else _FakeContext()
        self.created_clients = []  # [(srv_type, srv_name), ...] requested via create_client()
        self.client_instances = []  # [_FakeListControllersClient, ...] returned, in order
        self.destroyed_clients = []  # [_FakeListControllersClient, ...] passed to destroy_client()

    @property
    def client(self):
        """The most recently created client, for tests using a single ControllerLister."""
        return self.client_instances[-1]

    def create_client(self, srv_type, srv_name):
        self.created_clients.append((srv_type, srv_name))
        client = _FakeListControllersClient()
        self.client_instances.append(client)
        return client

    def destroy_client(self, client):
        self.destroyed_clients.append(client)
        return True


class _RaisingClient:
    def __init__(self, exc):
        self._exc = exc

    def call_async(self, request):
        raise self._exc


class _RaisingClientNode:
    def __init__(self, exc):
        self._exc = exc

    def create_client(self, srv_type, srv_name):
        return _RaisingClient(self._exc)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def _no_spin(monkeypatch):
    """
    Fail loudly if ControllerLister.__call__() ever calls rclpy.spin_once()
    or rclpy.spin_until_future_complete().

    It must not: node may be context.node, which rqt_gui_py's RclpySpinner
    already spins continuously on another thread. spin_once() there would
    race it, and spin_until_future_complete(node, future) -- with no executor
    argument -- silently detaches node from that other executor's node set
    (verified experimentally: node.executor gets reassigned to the global
    executor, permanently removing it from RclpySpinner's).
    """
    spin_once = Mock(side_effect=AssertionError("spin_once must not be called"))
    spin_until_future_complete = Mock(
        side_effect=AssertionError("spin_until_future_complete must not be called")
    )
    monkeypatch.setattr(utils.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(utils.rclpy, "spin_until_future_complete", spin_until_future_complete)
    return spin_once, spin_until_future_complete


@pytest.fixture
def _rclpy_ok(monkeypatch):
    """Make utils.rclpy.ok() report True without requiring a real rclpy.init()."""
    monkeypatch.setattr(utils.rclpy, "ok", lambda: True)


# ---------------------------------------------------------------------------
# get_controller_managers() / ControllerManagerLister: node injection
# ---------------------------------------------------------------------------


def test_get_controller_managers_uses_injected_node():
    node = _FakeGraphNode(_cm_services_under("/foo"))

    result = utils.get_controller_managers(node)

    assert result == ["/foo"]


def test_controller_manager_lister_requires_node_argument():
    with pytest.raises(TypeError):
        utils.ControllerManagerLister()


def test_controller_manager_lister_uses_injected_node(_rclpy_ok):
    node = _FakeGraphNode(_cm_services_under("/foo"))
    list_cm = utils.ControllerManagerLister(node)

    assert list_cm() == ["/foo"]


# ---------------------------------------------------------------------------
# ControllerLister: node injection, and Event-based (non-spinning) wait
# ---------------------------------------------------------------------------


def test_controller_lister_requires_node_argument():
    with pytest.raises(TypeError):
        utils.ControllerLister()


def test_controller_lister_creates_client_on_injected_node():
    node = _FakeControllerListerNode()

    utils.ControllerLister(node, "/controller_manager")

    assert node.created_clients == [
        (utils.ListControllers, "/controller_manager/list_controllers")
    ]


def test_controller_lister_call_waits_for_future_without_spinning(_no_spin):
    node = _FakeControllerListerNode()
    list_controllers = utils.ControllerLister(node)
    fake_result = SimpleNamespace(controller=["ctrl_a", "ctrl_b"])

    def deliver_after_delay():
        while node.client.last_future is None:
            time.sleep(0.01)
        time.sleep(0.05)
        node.client.last_future.complete(fake_result)

    thread = threading.Thread(target=deliver_after_delay, daemon=True)
    thread.start()

    result = list_controllers()
    thread.join(timeout=1.0)

    assert result == ["ctrl_a", "ctrl_b"]
    spin_once, spin_until_future_complete = _no_spin
    spin_once.assert_not_called()
    spin_until_future_complete.assert_not_called()


def test_controller_lister_call_returns_empty_list_on_shutdown_context_error():
    node = _RaisingClientNode(RuntimeError("context is not valid"))
    list_controllers = utils.ControllerLister(node)

    assert list_controllers() == []


def test_controller_lister_call_reraises_unexpected_errors():
    node = _RaisingClientNode(ValueError("boom"))
    list_controllers = utils.ControllerLister(node)

    with pytest.raises(ValueError, match="boom"):
        list_controllers()


# ---------------------------------------------------------------------------
# ControllerLister: service client lifetime (close()) and context-shutdown wait
# ---------------------------------------------------------------------------


def test_controller_lister_close_destroys_client_and_is_idempotent():
    node = _FakeControllerListerNode()
    list_controllers = utils.ControllerLister(node)
    client = node.client_instances[-1]

    list_controllers.close()
    list_controllers.close()  # must not double-destroy, or error, on repeat calls

    assert node.destroyed_clients == [client]


def test_repeated_controller_lister_replacement_does_not_accumulate_clients():
    """
    Mirrors what _on_cm_change() now does each time the user picks a different
    controller manager: close() the old ControllerLister before constructing
    a new one on the same (long-lived, shared) node. Node.create_client()
    never garbage-collects on its own, so without this, every replacement
    would leak one more client onto the shared node forever.
    """
    node = _FakeControllerListerNode()

    current = None
    for _ in range(5):
        if current is not None:
            current.close()
        current = utils.ControllerLister(node, "/controller_manager")

    assert len(node.client_instances) == 5
    assert len(node.destroyed_clients) == 4
    assert node.client_instances[-1] not in node.destroyed_clients


def test_controller_lister_call_stops_waiting_and_cleans_up_pending_request_on_context_shutdown(
    _no_spin,
):
    """
    While node.context.ok(), __call__() must keep waiting for a response
    indefinitely, same as the spin_until_future_complete() call it replaces.
    Once the node's own context goes not-ok (shutdown), it must stop waiting
    promptly rather than block forever on a future that will now never
    complete, fall back to an empty list like a call with no result, and --
    unlike a bare timeout -- must not leave the request sitting in the
    client's pending-requests table afterwards.
    """
    remaining_ok_checks = {"count": 2}

    class _ShuttingDownContext:
        def ok(self):
            remaining_ok_checks["count"] -= 1
            return remaining_ok_checks["count"] >= 0

    node = _FakeControllerListerNode(context=_ShuttingDownContext())
    list_controllers = utils.ControllerLister(node)

    result = list_controllers()

    assert result == []
    assert node.client.pending_requests == {}
    assert node.client.last_future.cancelled()
