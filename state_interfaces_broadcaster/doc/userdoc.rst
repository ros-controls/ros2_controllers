:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/state_interfaces_broadcaster/doc/userdoc.rst

.. _state_interfaces_broadcaster_userdoc:

State Interfaces Broadcaster
--------------------------------
The State Interfaces Broadcaster publishes the state of specified hardware interfaces of double data type.
The broadcaster publishes two topics:

- ``/state_interfaces_broadcaster/names``: Publishes the names of the hardware interfaces being monitored with a message type of ``control_msgs/msg/Keys``.
- ``/state_interfaces_broadcaster/values``: Publishes the current state values of the specified hardware interfaces with a message type of ``control_msgs/msg/Float64Values``.

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/state_interfaces_broadcaster/src/state_interfaces_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/state_interfaces_broadcaster_parameters.yaml


An example parameter file
=========================

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/state_interfaces_broadcaster/test/test_state_interfaces_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/test_state_interfaces_broadcaster_params.yaml
   :language: yaml
