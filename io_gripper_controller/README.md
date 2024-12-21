# IO Gripper Controller

The IO Gripper Controller is provides implementation to control the gripper using IOs. It provides functionalities like open, close and reconfigure which can be used either though action server or service server and also publishes `joint_states` of gripper and also `dynamic_interfaces` for all command and state interfaces.

## Description of controller's interfaces

- `joint_states` [`sensor_msgs::msg::JointState`]: Publishes the state of gripper joint and configuration joint
- `dynamic_interfaces` [`control_msgs::msg::DynamicInterfaceValues`]: Publishes all command and state interface of the IOs and sensors of gripper.

