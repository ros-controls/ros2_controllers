:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/doc/userdoc.rst

.. _diff_drive_controller_userdoc:

diff_drive_controller
=====================

Controller for mobile robots with differential drive.

As input it takes velocity commands for the robot body, which are translated to wheel commands for the differential drive base.

Odometry is computed from hardware feedback and published.

Other features
--------------

   + Realtime-safe implementation.
   + Odometry publishing
   + Task-space velocity, acceleration and jerk limits
   + Automatic stop after command time-out


Description of controller's interfaces
------------------------------------------------

References
,,,,,,,,,,,,,,,,,,

(the controller is not yet implemented as chainable controller)

Feedback
,,,,,,,,,,,,,,

As feedback interface type the joints' position (``hardware_interface::HW_IF_POSITION``) or velocity (``hardware_interface::HW_IF_VELOCITY``,if parameter ``position_feedback=false``) are used.

Output
,,,,,,,,,

Joints' velocity (``hardware_interface::HW_IF_VELOCITY``) are used.


ROS 2 Interfaces
------------------------

Subscribers
,,,,,,,,,,,,

~/cmd_vel [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller, if ``use_stamped_vel=true``. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.


Publishers
,,,,,,,,,,,
~/odom [nav_msgs::msg::Odometry]
  This represents an estimate of the robot's position and velocity in free space.

/tf [tf2_msgs::msg::TFMessage]
  tf tree. Published only if ``enable_odom_tf=true``

~/cmd_vel_out [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller, where limits were applied. Published only if ``publish_limited_velocity=true``


Parameters
,,,,,,,,,,,,

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
