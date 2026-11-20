#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
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

# NOTE: The Python API contained in this file is considered UNSTABLE and
# subject to change.
# No backwards compatibility guarantees are provided at this moment.


import time

import rclpy
from controller_manager_msgs.srv import ListControllers

# Names of controller manager services, and their respective types
_LIST_CONTROLLERS_STR = "list_controllers"
_LIST_CONTROLLERS_TYPE = "controller_manager_msgs/srv/ListControllers"
_LIST_CONTROLLER_TYPES_STR = "list_controller_types"
_LIST_CONTROLLER_TYPES_TYPE = "controller_manager_msgs/srv/ListControllerTypes"
_LOAD_CONTROLLER_STR = "load_controller"
_LOAD_CONTROLLER_TYPE = "controller_manager_msgs/srv/LoadController"
_UNLOAD_CONTROLLER_STR = "unload_controller"
_UNLOAD_CONTROLLER_TYPE = "controller_manager_msgs/srv/UnloadController"
_SWITCH_CONTROLLER_STR = "switch_controller"
_SWITCH_CONTROLLER_TYPE = "controller_manager_msgs/srv/SwitchController"
_RELOAD_CONTROLLER_LIBS_STR = "reload_controller_libraries"
_RELOAD_CONTROLLER_LIBS_TYPE = "controller_manager_msgs/srv/" "ReloadControllerLibraries"

# Map from service names to their respective type
cm_services = {
    _LIST_CONTROLLERS_STR: _LIST_CONTROLLERS_TYPE,
    _LIST_CONTROLLER_TYPES_STR: _LIST_CONTROLLER_TYPES_TYPE,
    _LOAD_CONTROLLER_STR: _LOAD_CONTROLLER_TYPE,
    _UNLOAD_CONTROLLER_STR: _UNLOAD_CONTROLLER_TYPE,
    _SWITCH_CONTROLLER_STR: _SWITCH_CONTROLLER_TYPE,
    _RELOAD_CONTROLLER_LIBS_STR: _RELOAD_CONTROLLER_LIBS_TYPE,
}


def is_shutdown_context_error(exc):
    message = str(exc)
    return (
        "context is not valid" in message
        or "context is invalid" in message
        or "destruction was requested" in message
    )


def get_controller_managers(node, namespace="/", initial_guess=None):
    """
    Get list of active controller manager namespaces.

    @param node: Node used to query the ROS graph for controller_manager services.
    Only synchronous graph introspection calls are made on it (get_service_names_and_types);
    it is never spun, so it is safe to pass a node whose spinning is owned elsewhere
    (e.g. rqt's shared context.node).
    @type node: rclpy.node.Node
    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @param initial_guess: Initial guess of the active controller managers.
    Typically c{initial_guess} is the output of a previous call to this method,
    and is useful when periodically checking for changes in the list of
    active controller managers.
    Elements in this list will go through a lazy validity check (as opposed to
    a full name+type API verification), so providing a good estimate can
    significantly reduce the number of ROS master queries incurred by this
    method.
    @type initial_guess: [str]
    @return: Sorted list of active controller manager namespaces.
    @rtype: [str]
    """
    ns_list = []
    if initial_guess is not None:
        ns_list = initial_guess[:]  # force copy

    # Get list of (potential) currently running controller managers
    ns_list_curr = _sloppy_get_controller_managers(node, namespace)

    # Update initial guess:
    # 1. Remove entries not found in current list
    # 2. Add new untracked controller managers
    ns_list[:] = [ns for ns in ns_list if ns in ns_list_curr]
    ns_list += [ns for ns in ns_list_curr if ns not in ns_list and is_controller_manager(node, ns)]

    return sorted(ns_list)


def is_controller_manager(node, namespace):
    """
    Check if the input namespace exposes the controller_manager ROS interface.

    This method has the overhead of several ROS master queries
    (one per ROS API member).

    @param namespace: Namespace to check
    @type namespace: str
    @return: True if namespace exposes the controller_manager ROS interface
    @rtype: bool
    """
    cm_ns = namespace
    if not cm_ns or cm_ns[-1] != "/":
        cm_ns += "/"
    for srv_name in cm_services.keys():
        if not _srv_exists(node, cm_ns + srv_name, cm_services[srv_name]):
            return False
    return True


def _sloppy_get_controller_managers(node, namespace):
    """
    Get list of I{potential} active controller manager namespaces.

    The method name is prepended with I{sloppy}, and the returned list contains
    I{potential} active controller managers because it does not perform a
    thorough check of the expected ROS API.
    It rather tries to minimize the number of ROS master queries.

    This method has the overhead of one ROS master query.

    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @return: List of I{potential} active controller manager namespaces.
    @rtype: [str]
    """
    # refresh the list of controller managers we can find
    srv_list = node.get_service_names_and_types()

    ns_list = []
    for srv_info in srv_list:
        match_str = "/" + _LIST_CONTROLLERS_STR
        # First element of srv_name is the service name
        if match_str in srv_info[0]:
            ns = srv_info[0].split(match_str)[0]
            if ns == "":
                # controller manager services live in root namespace
                # unlikely, but still possible
                ns = "/"
            ns_list.append(ns)
    return ns_list


def _srv_exists(node, srv_name, srv_type):
    """
    Check if a ROS service of specific name and type exists.

    This method has the overhead of one ROS master query.

    @param srv_name: Fully qualified service name
    @type srv_name:  str
    @param srv_type: Service type
    @type srv_type: str
    """
    if not srv_name or not srv_type:
        return False

    srv_list = node.get_service_names_and_types()
    srv_info = [item for item in srv_list if item[0] == srv_name]
    if len(srv_info) == 0:
        return False
    srv_obtained_type = srv_info[0][1][0]
    return srv_type == srv_obtained_type


###############################################################################
#
# Convenience classes for querying controller managers and controllers
#
###############################################################################


class ControllerManagerLister:
    """
    Convenience functor for querying the list of active controller managers.

    Useful when frequently updating the list, as it internally performs
    some optimizations that reduce the number of interactions with the
    ROS master.

    Example usage:
        >>> list_cm = ControllerManagerLister(node)
        >>> print(list_cm())
    """

    def __init__(self, node, namespace="/"):
        """
        Store the injected node and namespace used to look up controller managers.

        @param node Node used to query the ROS graph. Never spun by this class.
        @type node rclpy.node.Node
        @param namespace Namespace where to look for controller managers.
        @type namespace str
        """
        self._node = node
        self._ns = namespace
        self._cm_list = []

    def __call__(self):
        """Get list of running controller managers."""
        if not rclpy.ok():
            return self._cm_list

        try:
            self._cm_list = get_controller_managers(self._node, self._ns, self._cm_list)
        except rclpy.executors.ExternalShutdownException:
            return self._cm_list
        except Exception as e:
            if is_shutdown_context_error(e):
                return self._cm_list
            raise
        return self._cm_list


class ControllerLister:
    """
    Convenience functor for querying loaded controller data.

    The output of calling this functor can be used as input to the different
    controller filtering functions available in this module.

    Example usage. Get I{running} controllers of type C{bar_base/bar}:
        >>> list_controllers = ControllerLister(node, 'foo_robot/controller_manager')
        >>> all_ctrl = list_controllers()
        >>> running_ctrl = filter_by_state(all_ctrl, 'running')
        >>> running_bar_ctrl = filter_by_type(running_ctrl, 'bar_base/bar')
    """

    def __init__(self, node, namespace="/controller_manager", timeout_sec=2.0):
        """
        Store the injected node and namespace, and create the list_controllers client.

        @param node Node used to call the controller manager's list_controllers
        service. This class never spins node itself (see __call__): node may be
        a shared node (e.g. rqt's context.node) that some other executor already
        spins continuously elsewhere, and calling rclpy.spin_until_future_complete()
        or spin_once() on it here would race against that executor.
        @type node rclpy.node.Node

        @param namespace Namespace of controller manager to monitor.
        @type namespace str

        @param timeout_sec Time (in seconds) after which a request without response is
        dropped and the controller list becomes unknown.
        @type timeout_sec float
        """
        self._node = node
        self._timeout_sec = timeout_sec
        self._srv_name = namespace + "/" + _LIST_CONTROLLERS_STR
        self._srv_client = self._create_client()
        self._future = None  # pending list_controllers request
        self._sent_time = 0.0  # time the pending request was sent
        self._controllers = None  # last received controller list, None if unknown

    def __call__(self):
        """
        Return the last received controller list and request an update, without blocking.

        Called periodically from the GUI thread. The request is answered via the executor
        spinning node (e.g., rqt's), so its response is returned by a later call.

        @return: Controller list, [] if the controller manager is not available, or None if
        unknown, i.e., no response was received yet or the last request timed out.
        @rtype: [controller_manager_msgs/ControllerState] or None
        """
        try:
            if self._future is not None:
                if self._future.done():
                    result = self._future.result()
                    self._controllers = result.controller if result else None
                    self._future = None
                elif time.monotonic() - self._sent_time > self._timeout_sec:
                    self._future.cancel()
                    self._future = None
                    self._controllers = None
            if self._future is None:
                if self._srv_client.service_is_ready():
                    self._future = self._srv_client.call_async(ListControllers.Request())
                    self._sent_time = time.monotonic()
                else:
                    # A request to a missing controller manager would never be answered
                    self._controllers = []
            return self._controllers
        except rclpy.executors.ExternalShutdownException:
            return []
        except Exception as e:
            if is_shutdown_context_error(e):
                return []
            raise

    def _create_client(self):
        return self._node.create_client(ListControllers, self._srv_name)

    def close(self):
        """
        Destroy the list_controllers client this instance owns on self._node.

        Node.create_client() registers the client on self._node and never garbage-
        collects it on its own; the client only stops being registered once
        self._node.destroy_client() is called on it, or self._node itself is
        destroyed. Since self._node may be long-lived and shared (e.g. rqt's
        context.node), callers must call close() once this ControllerLister is no
        longer used -- e.g. before replacing it with a new one, when
        controller-manager selection is cleared, or during plugin shutdown --
        otherwise every replacement leaks one more client onto self._node forever.
        Does not touch self._node itself.
        """
        if self._srv_client is not None:
            self._node.destroy_client(self._srv_client)
            self._srv_client = None


###############################################################################
#
# Convenience methods for filtering controller state information
#
###############################################################################


def filter_by_type(ctrl_list, ctrl_type, match_substring=False):
    """
    Filter controller state list by controller type.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_type: Controller type
    @type ctrl_type: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified type
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "type", ctrl_type, match_substring)


def filter_by_state(ctrl_list, ctrl_state, match_substring=False):
    """
    Filter controller state list by controller state.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_state: Controller state
    @type ctrl_state: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified state
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "state", ctrl_state, match_substring)


def _filter_by_attr(list_in, attr_name, attr_val, match_substring=False):
    """Filter input list by the value of its elements' attributes."""
    list_out = []
    for val in list_in:
        if match_substring:
            if attr_val in getattr(val, attr_name):
                list_out.append(val)
        else:
            if getattr(val, attr_name) == attr_val:
                list_out.append(val)
    return list_out
