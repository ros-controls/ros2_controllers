:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/imu_sensor_broadcaster/doc/userdoc.rst

.. _imu_sensor_broadcaster_userdoc:

IMU Sensor Broadcaster
--------------------------------
Broadcaster of messages from IMU sensor.
The published message type is ``sensor_msgs/msg/Imu``.

The controller is a wrapper around ``IMUSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/imu_sensor_broadcaster/src/imu_sensor_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/imu_sensor_broadcaster_parameters.yaml


An example parameter file
=========================

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/imu_sensor_broadcaster/test/imu_sensor_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/imu_sensor_broadcaster_params.yaml
   :language: yaml
