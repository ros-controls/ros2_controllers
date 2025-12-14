:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/interfaces_state_broadcaster/doc/userdoc.rst

.. _interfaces_state_broadcaster_userdoc:

Interfaces State Broadcaster
--------------------------------
The Interfaces State Broadcaster publishes the state of specified hardware interfaces that support double data type.
The broadcaster publishes two topics:

- ``/interfaces_state_broadcaster/names``: Publishes the names of the hardware interfaces being monitored with a message type of ``control_msgs/msg/Keys``.
- ``/interfaces_state_broadcaster/values``: Publishes the current state values of the specified hardware interfaces with a message type of ``control_msgs/msg/Float64Values``.

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/interfaces_state_broadcaster/src/interfaces_state_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/interfaces_state_broadcaster_parameters.yaml


An example parameter file
=========================

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/interfaces_state_broadcaster/test/test_interfaces_state_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/test_interfaces_state_broadcaster_params.yaml
   :language: yaml
