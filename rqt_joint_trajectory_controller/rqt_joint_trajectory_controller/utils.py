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


import threading

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

    def __init__(self, node, namespace="/controller_manager"):
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
        """
        self._node = node
        self._srv_name = namespace + "/" + _LIST_CONTROLLERS_STR
        self._srv_client = self._create_client()

    """
    @return: Controller list.
    @rtype: [controller_manager_msgs/ControllerState]
    """

    def __call__(self):
        try:
            future = self._srv_client.call_async(ListControllers.Request())
            # Wait via a plain threading.Event rather than spinning self._node:
            # self._node may be context.node, which rqt_gui_py's RclpySpinner
            # already spins continuously on another thread. Its executor already
            # processes this future's response and completes it; we only need to
            # be woken up when that happens. Poll self._node's own context between
            # short waits so shutdown doesn't block here indefinitely, mirroring
            # the exit condition spin_until_future_complete used to provide.
            # self._node.context.ok() (not rclpy.ok(), which always checks the
            # process-wide default context) is the correct check here: self._node
            # may not be tied to the default context, and it is that node's
            # context -- not the default one -- whose executor is what would ever
            # complete this future.
            #
            # We deliberately do not use Client.call(): in Humble it blocks with
            # no timeout at all and no way to observe context shutdown, and even
            # in Jazzy/Rolling (where it gained a timeout_sec) a fixed wall-clock
            # timeout can't express "keep waiting while healthy, stop promptly on
            # shutdown" -- polling it in a loop would just resend a fresh request
            # each iteration, since it calls call_async() internally every time.
            done_event = threading.Event()
            future.add_done_callback(lambda _future: done_event.set())
            while self._node.context.ok() and not done_event.is_set():
                done_event.wait(timeout=0.1)
            # No-op if the future is already done. Otherwise (we gave up waiting
            # because the context is shutting down) this marks it CANCELLED,
            # which runs Client.remove_pending_request() via the future's own
            # done-callback, so the request is not left registered in the
            # client's pending-requests table after we stop waiting on it.
            future.cancel()
            result = future.result()
            return result.controller if result else []
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


def filter_by_name(ctrl_list, ctrl_name, match_substring=False):
    """
    Filter controller state list by controller name.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_name: Controller name
    @type ctrl_name: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified name
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, "name", ctrl_name, match_substring)


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


def filter_by_hardware_interface(ctrl_list, hardware_interface, match_substring=False):
    """
    Filter controller state list by controller hardware interface.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param hardware_interface: Controller hardware interface
    @type hardware_interface: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    list_out = []
    for ctrl in ctrl_list:
        for resource_set in ctrl.claimed_resources:
            if match_substring:
                if hardware_interface in resource_set.hardware_interface:
                    list_out.append(ctrl)
                    break
            else:
                if resource_set.hardware_interface == hardware_interface:
                    list_out.append(ctrl)
                    break
    return list_out


def filter_by_resources(ctrl_list, resources, hardware_interface=None, match_any=False):
    """
    Filter controller state list by claimed resources.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param resources: Controller resources
    @type resources: [str]
    @param hardware_interface Controller hardware interface where to look for
    resources. If specified, the requested resources will only be searched for
    in this interface. If unspecified, all controller hardware interfaces will
    be searched for; i.e., if a controller claims resources from multiple
    interfaces, the method will succeed if _any_ interface contains the
    requested resources (any or all, depending on the value of C{match_any}).
    Specifying this parameter allows finer control over determining which
    interfaces claim specific resources.
    @param match_any: If set to False, all elements in C{resources} must
    be claimed by the interface specified in C{hardware_interface} (or _any_
    interface, if C{hardware_interface} is unspecified) for a positive match.
    Note that a controller's resources can contain additional entries than
    those in C{resources}).
    If set to True, at least one element in C{resources} must be claimed by
    the interface specified in C{hardware_interface} (or _any_ interface, if
    C{hardware_interface} is unspecified) for a positive match.
    @type match_any: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    list_out = []
    for ctrl in ctrl_list:
        for resource_set in ctrl.claimed_resources:
            if hardware_interface is None or hardware_interface == resource_set.hardware_interface:
                for res in resources:
                    add_ctrl = not match_any  # Initial flag value
                    if res in resource_set.resources:
                        if match_any:  # One hit: enough to accept controller
                            add_ctrl = True
                            break
                    else:
                        if not match_any:  # One miss: enough to discard controller
                            add_ctrl = False
                            break
                if add_ctrl:
                    list_out.append(ctrl)
                    break
    return list_out


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


###############################################################################
#
# Convenience methods for finding potential controller configurations
#
###############################################################################

# def get_rosparam_controller_names(namespace='/'):
#     """
#     Get list of ROS parameter names that potentially represent a controller
#     configuration.

#     Example usage:
#       - Assume the following parameters exist in the ROS parameter:
#         server:
#           - C{/foo/type}
#           - C{/xxx/type/xxx}
#           - C{/ns/bar/type}
#           - C{/ns/yyy/type/yyy}
#       - The potential controllers found by this method are:

#       >>> names    = get_rosparam_controller_names()      # returns ['foo']
#       >>> names_ns = get_rosparam_controller_names('/ns') # returns ['bar']

#     @param namespace: Namespace where to look for controllers.
#     @type namespace: str
#     @return: Sorted list of ROS parameter names.
#     @rtype: [str]
#     """
#     import rosparam
#     list_out = []
#     all_params = rosparam.list_params(namespace)
#     for param in all_params:
#         # Remove namespace from parameter string
#         if not namespace or namespace[-1] != '/':
#             namespace += '/'
#         param_no_ns = param.split(namespace, 1)[1]

#         # Check if parameter corresponds to a controller configuration
#         param_split = param_no_ns.split('/')
#         if (len(param_split) == 2 and param_split[1] == 'type'):
#             list_out.append(param_split[0]) # It does!
#     return sorted(list_out)
