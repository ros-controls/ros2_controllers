.. _diff_drive_controller_userdoc:

diff_drive_controller
=====================

Controller for mobile robots with differential drive.
Input for control are robot body velocity commands which are translated to wheel commands for the differential drive base.
Odometry is computed from hardware feedback and published.

Velocity commands
-----------------

The controller works with a velocity twist from which it extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Hardware interface type
-----------------------

The controller works with wheel joints through a velocity interface.

Other features
--------------

    Realtime-safe implementation.
    Odometry publishing
    Task-space velocity, acceleration and jerk limits
    Automatic stop after command time-out


ros2_control Interfaces
------------------------

References
,,,,,,,,,,,


States
,,,,,,,


Commands
,,,,,,,,,


ROS2 Interfaces
----------------

Subscribers
,,,,,,,,,,,,
~/cmd_vel [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller.

~/cmd_vel_unstamped [geometry_msgs::msg::Twist]

~/cmd_vel_out []




Publishers
,,,,,,,,,,,
~/odom []

/tf


Services
,,,,,,,,,


Parameters
------------

Wheels
,,,,,,,
left_wheel_names [array<string>]
  Link names of the left side wheels.

right_wheel_names  [array<string>]
  Link names of the right side wheels.

wheel_separation [double]
  Shortest distance between the left and right wheels.
  If this parameter is wrong, the robot will not behave correctly in curves.

wheels_per_side [integer]
  Number of wheels on each wide of the robot.
  This is important to take the wheels slip into account when multiple wheels on each side are present.
  If there are more wheels then control signals for each side, you should enter number or control signals.
  For example, Husky has two wheels on each side, but they use one control signal, in this case "1" is the correct value of the parameter.

wheel_radius [double]
  Radius of a wheel, i.e., wheels size, used for transformation of linear velocity into wheel rotations.
  If this parameter is wrong the robot will move faster or slower then expected.

wheel_separation_multiplier [double]
  Correction factor for wheel separation (TODO(destogl): Please help me describe this correctly)

left_wheel_radius_multiplier [double]
  Correction factor when radius of left wheels differs from the nominal value in ``wheel_radius`` parameter.

right_wheel_radius_multiplier [double]
  Correction factor when radius of right wheels differs from the nominal value in ``wheel_radius`` parameter.

Odometry
,,,,,,,,,
base_frame_id [string] (default: "base_link")
  Name of the frame for odometry.
  This frame is parent of ``base_frame_id`` when controller publishes odometry.

odom_frame_id [string] (default: "odom")
  Name of the robot's base frame that is child of the odometry frame.

pose_covariance_diagonal [array<double>[6]] (default: "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
  Odometry covariance for the encoder output of the robot for the pose.
  These values should be tuned to your robot's sample odometry data, but these values are a good place to start:
  ``[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]``.

twist_covariance_diagonal [array<double>[6]] (default: "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
  Odometry covariance for the encoder output of the robot for the speed.
  These values should be tuned to your robot's sample odometry data, but these values are a good place to start:
  ``[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]``.

open_loop [bool] (default: "false")
  If set to false the odometry of the robot will be calculated from the commanded values and not from feedback.

position_feedback [bool] (default: "true")
  Is there position feedback from hardware.

enable_odom_tf [bool] (default: "true")
  Publish transformation between ``odom_frame_id`` and ``base_frame_id``.

velocity_rolling_window_size [int] (default: "10")
  (TODO(destogl): Please help me describe this correctly)

Commands
```````````
cmd_vel_timeout [double] (default: "0.5s")
  Timeout after which input command on ``cmd_vel`` topic is considered staled.

publish_limited_velocity [double] (default: "false")
  Publish limited velocity value.


use_stamped_vel [bool] (default: "true")
  Use stamp from input velocity message to calculate how old the command actually is.

linear.x [JointLimits structure]
  Joint limits structure for the linear X-axis.
  The limiter ignores position limits.
  For details see ``joint_limits`` package from ros2_control repository.

angular.z [JointLimits structure]
  Joint limits structure for the rotation about Z-axis.
  The limiter ignores position limits.
  For details see ``joint_limits`` package from ros2_control repository.

publish_rate [double] (default: "50.0")
  Publishing rate (Hz) of the odometry and TF messages.
