:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/doc/parameters.rst

.. _parameters:

Details about parameters
^^^^^^^^^^^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/src/joint_trajectory_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


List of parameters
=========================

.. generate_parameter_library_details::
  ../src/joint_trajectory_controller_parameters.yaml
  parameters_context.yaml

An example parameter file
=========================

.. generate_parameter_library_default::
  ../src/joint_trajectory_controller_parameters.yaml
