.. _swerve_drive_controller_userdoc:

swerve_drive_controller
=========================

Library with shared functionalities for mobile robot controllers with swerve drive (four swerve wheels).
The library implements generic odometry and update methods and defines the main interfaces.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped or non stamped Twist messages where linear ``x``, ``y``, and angular ``z`` components are used.
Values in other components are ignored.

Note about odometry calculation:
In the DiffDRiveController, the velocity is filtered out, but we prefer to return it raw and let the user perform post-processing at will.
We prefer this way of doing so as filtering introduces delay (which makes it difficult to interpret and compare behavior curves).


Description of controller's interfaces
--------------------------------------

Subscribers
,,,,,,,,,,,,

If ``use_stamped_vel=true``:
~/cmd_vel [geometry_msgs/msg/TwistStamped]  
  Velocity command for the controller. The controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

If ``use_stamped_vel=false``:
~/cmd_vel [geometry_msgs/msg/Twist]  
  Velocity command for the controller. The controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Publishers
,,,,,,,,,,,
~/odom [nav_msgs::msg::Odometry]
  This represents an estimate of the robot's position and velocity in free space.

/tf [tf2_msgs::msg::TFMessage]
  tf tree. Published only if ``enable_odom_tf=true``


Parameters
,,,,,,,,,,,

.. literalinclude:: ../test/config/test_swerve_drive_controller.yaml
   :language: yaml
