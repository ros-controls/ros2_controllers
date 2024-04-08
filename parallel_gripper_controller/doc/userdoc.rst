:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/parallel_gripper_controller/doc/userdoc.rst

.. _parallel_gripper_controller_userdoc:

Parallel Gripper Action Controller
-----------------------------------

Controller for executing a ``ParallelGripperCommand`` action for simple parallel grippers.
This controller supports grippers that offer position only control as well as grippers that allow configuring the velocity and effort.
By default, the controller will only claim the ``{joint}/position`` interface for control.
The velocity and effort interfaces can be optionally claimed by setting the ``max_velocity_interface`` and ``max_effort_interface`` parameter, respectively.
By default, the controller will try to claim position and velocity state interfaces.
The claimed state interfaces can be configured by setting the ``state_interfaces`` parameter.

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

.. generate_parameter_library_details:: ../src/gripper_action_controller_parameters.yaml
