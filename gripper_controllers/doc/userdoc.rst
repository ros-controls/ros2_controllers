:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gripper_controllers/doc/userdoc.rst

.. _gripper_controllers_userdoc:

Gripper Action Controller
--------------------------------

Controllers for executing a gripper command action for simple single-dof grippers:

- ``position_controllers/GripperActionController``
- ``effort_controllers/GripperActionController``

Parameters
^^^^^^^^^^^
These controllers use the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gripper_controllers/src/gripper_action_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

List of parameters
=========================

.. generate_parameter_library_details:: ../src/gripper_action_controller_parameters.yaml


An example parameter file
=========================

.. generate_parameter_library_default::
  ../src/gripper_action_controller_parameters.yaml
