:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/imu_sensor_broadcaster/doc/userdoc.rst

.. _imu_sensor_broadcaster_userdoc:

IMU Sensor Broadcaster
--------------------------------
Broadcaster of messages from IMU sensor.
The published message type is ``sensor_msgs/msg/Imu``.

The controller is a wrapper around ``IMUSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

.. generate_parameter_library_details:: ../src/imu_sensor_broadcaster_parameters.yaml
