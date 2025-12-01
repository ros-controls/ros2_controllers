force_torque_sensor_broadcaster
==========================================

Controller to publish state of force-torque sensors.

Pluginlib-Library: force_torque_sensor_broadcaster

Plugin: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster (controller_interface::ControllerInterface)

Wrench Transformer Node
-----------------------
The package also provides a standalone ROS 2 node ``wrench_transformer_node`` that transforms wrench messages from the force_torque_sensor_broadcaster to different target frames using TF2. This allows applications to receive force/torque data in their preferred coordinate frames.

See the user documentation for details on configuration and usage.
