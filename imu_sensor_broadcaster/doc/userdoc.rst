.. _imu_sensor_broadcaster_userdoc:

IMU Sensor Broadcaster
--------------------------------
Broadcaster of messages from IMU sensor.
The published message type is ``sensor_msgs/msg/Imu``.

The controller is a wrapper around ``IMUSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
frame_id (mandatory)
  Frame in which the output message will be published.

sensor_name (mandatory)
  Defines sensor name used as prefix for its interfaces.
  Interface names are: <sensor_name>/orientation.x, ..., <sensor_name>/angular_velocity.x, ...,
  <sensor_name>/linear_acceleration.x.
