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

~/cmd_vel [geometry_msgs/msg/TwistStamped] (If ``use_stamped_vel=true``)
  Velocity command for the controller. The controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

~/cmd_vel [geometry_msgs/msg/Twist] (If ``use_stamped_vel=false``)
  Velocity command for the controller. The controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Publishers
,,,,,,,,,,,
~/odom [nav_msgs::msg::Odometry]
  This represents an estimate of the robot's position and velocity in free space.

/tf [tf2_msgs::msg::TFMessage]
  tf tree. Published only if ``enable_odom_tf=true``


Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/swerve_drive_controller/src/swerve_drive_controller_parameter.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/swerve_drive_controller_parameter.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/swerve_drive_controller/test/config/test_swerve_drive_controller.yaml>`_:

.. literalinclude:: ../test/config/test_swerve_drive_controller.yaml
   :language: yaml
