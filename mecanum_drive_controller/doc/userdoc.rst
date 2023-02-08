.. _mecanum_drive_controller_userdoc:

mecanum_drive_controller
=========================

Library with shared functionalities for mobile robots controllers with mecanum drive (four wheels).
The library implements generic odometry and update methods and defines the main interfaces.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped or unstamped Twist messages where linear ``x``, ``y``, and angular ``z`` components are used.
Values in other components are ignored.
In the chain mode the controller provides three reference interfaces, one for linear velocity and one for steering angle position.
Other relevant features are:

  - odometry publishing as Odometry and TF message;
  - input command timeout based on a parameter.


Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
- <reference_names[i]>/<interface_name>  [double]  # in [rad] or [rad/s]

Commands
,,,,,,,,,
- <command_joint_names[i]>/<interface_name>  [double]  # in [rad] or [rad/s]

States
,,,,,,,
- <joint_names[i]>/<interface_name>  [double]  # in [rad] or [rad/s]
  **NOTE**: ``joint_name`` can be ``state_joint_names`` parameter of if empty ``command_joint_names``.


Subscribers
,,,,,,,,,,,,
Used when controller is not in chained mode (``in_chained_mode == false``).

- <controller_name>/reference  [geometry_msgs/msg/TwistStamped]

Publishers
,,,,,,,,,,,
- <controller_name>/odometry          [nav_msgs/msg/Odometry]
- <controller_name>/tf_odometry       [tf2_msgs/msg/TFMessage]
- <controller_name>/controller_state  [control_msgs/msg/MecanumDriveControllerState]

Parameters
,,,,,,,,,,,

For list of parameters and their meaning YAML file in the ``src`` folder of the controller's package.

For an exameplary parameterization see the ``test`` folder of the controller's package.
