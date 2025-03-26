:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/doc/parameters.rst

.. _parameters:

Details about parameters
^^^^^^^^^^^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/src/joint_trajectory_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


List of parameters
=========================

joint_trajectory_controller
---------------------------

.. generate_parameter_library_details::
  ../src/joint_trajectory_controller_parameters.yaml
  parameters_context.yaml

pid_trajectory_plugin
-----------------------------------

.. generate_parameter_library_details::
  ../../joint_trajectory_controller_plugins/src/pid_trajectory_plugin_parameters.yaml
  ../../joint_trajectory_controller_plugins/doc/parameters_context.yaml

An example parameter file
=========================

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/test/test_joint_trajectory_controller_params.yaml>`_:

.. literalinclude:: ../test/test_joint_trajectory_controller_params.yaml
   :language: yaml
