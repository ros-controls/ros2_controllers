:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/doc/userdoc.rst

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

Check `parameter definition file for details <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/src/diff_drive_controller_parameter.yaml>`_.

Note that the documentation on parameters for joint limits can be found in `their header file <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/joint_limits/include/joint_limits/joint_limits_rosparam.hpp#L56-L75>`_.
Those parameters are:

linear.x [JointLimits structure]
  Joint limits structure for the linear X-axis.
  The limiter ignores position limits.
  For details see ``joint_limits`` package from ros2_control repository.

angular.z [JointLimits structure]
  Joint limits structure for the rotation about Z-axis.
  The limiter ignores position limits.
  For details see ``joint_limits`` package from ros2_control repository.
