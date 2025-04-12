:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/multi_omni_wheel_drive_controller/doc/userdoc.rst

.. _multi_omni_wheel_drive_controller_userdoc:

multi_omni_wheel_drive_controller
=================================

Controller for mobile robots with omnidirectional drive.

Supports using three or more omni wheels spaced at an equal angle from each other in a circular formation.
To better understand this, have a look at :ref:`mobile_robot_kinematics`.

The controller uses velocity input, i.e., stamped Twist messages where linear ``x``, ``y``, and angular ``z`` components are used.
Values in other components are ignored.

Odometry is computed from hardware feedback and published.

Other features
--------------

   + Realtime-safe implementation.
   + Odometry publishing
   + Automatic stop after command time-out

Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

When controller is in chained mode, it exposes the following references which can be commanded by the preceding controller:

- ``<controller_name>/linear/x/velocity`` double, in m/s
- ``<controller_name>/linear/y/velocity`` double, in m/s
- ``<controller_name>/angular/z/velocity`` double, in rad/s

Together, these represent the body twist (which in unchained-mode would be obtained from ~/cmd_vel).

State interfaces
,,,,,,,,,,,,,,,,

As feedback interface type the joints' position (``hardware_interface::HW_IF_POSITION``) or velocity (``hardware_interface::HW_IF_VELOCITY``, if parameter ``position_feedback=false``) are used.

Command interfaces
,,,,,,,,,,,,,,,,,,,,,,

Joints' velocity (``hardware_interface::HW_IF_VELOCITY``) are used.


ROS 2 Interfaces
----------------

Subscribers
,,,,,,,,,,,

~/cmd_vel [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller. The controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Publishers
,,,,,,,,,,
~/odom [nav_msgs::msg::Odometry]
  This represents an estimate of the robot's position and velocity in free space.

/tf [tf2_msgs::msg::TFMessage]
  tf tree. Published only if ``enable_odom_tf=true``


Parameters
,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/multi_omni_wheel_drive_controller/src/multi_omni_wheel_drive_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/multi_omni_wheel_drive_controller_parameters.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/multi_omni_wheel_drive_controller/test/config/test_multi_omni_wheel_drive_controller.yaml>`_:

.. literalinclude:: ../test/config/test_multi_omni_wheel_drive_controller.yaml
   :language: yaml
