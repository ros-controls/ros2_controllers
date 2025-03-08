.. _mecanum_drive_controller_userdoc:

mecanum_drive_controller
=========================

Library with shared functionalities for mobile robot controllers with mecanum drive (four wheels).
The library implements generic odometry and update methods and defines the main interfaces.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped Twist messages where linear ``x``, ``y``, and angular ``z`` components are used.
Values in other components are ignored.
In the chain mode, the controller provides three reference interfaces, one for linear velocity and one for steering angle position.
Other relevant features are:

  - odometry publishing as Odometry and TF message;
  - input command timeout based on a parameter.

Note about odometry calculation:
In the DiffDRiveController, the velocity is filtered out, but we prefer to return it raw and let the user perform post-processing at will.
We prefer this way of doing so as filtering introduces delay (which makes it difficult to interpret and compare behavior curves).


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
  ..note ::

  ``joint_names[i]`` can be of ``state_joint_names`` parameter (if used), ``command_joint_names`` otherwise.


Subscribers
,,,,,,,,,,,,
Used when the controller is not in chained mode (``in_chained_mode == false``).

- <controller_name>/reference  [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller, if ``use_stamped_vel=true``. 

- <controller_name>/reference_unstamped  [geometry_msgs/msg/Twist]
  Velocity command for the controller, if ``use_stamped_vel=false``.

Publishers
,,,,,,,,,,,
- <controller_name>/odometry          [nav_msgs/msg/Odometry]
- <controller_name>/tf_odometry       [tf2_msgs/msg/TFMessage]
- <controller_name>/controller_state  [control_msgs/msg/MecanumDriveControllerState]

Parameters
,,,,,,,,,,,

For a list of parameters and their meaning, see the YAML file in the ``src`` folder of the controller's package.

For an exemplary parameterization, see the ``test`` folder of the controller's package.
