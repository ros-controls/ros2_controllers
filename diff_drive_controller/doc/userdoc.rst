:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/doc/userdoc.rst

.. _diff_drive_controller_userdoc:

diff_drive_controller
=====================

Controller for mobile robots with differential drive.

As input it takes velocity commands for the robot body, which are translated to wheel commands for the differential drive base.

Odometry is computed from hardware feedback and published.

For an introduction to mobile robot kinematics and the nomenclature used here, see :ref:`mobile_robot_kinematics`.

Other features
--------------

   + Realtime-safe implementation.
   + Odometry publishing
   + Task-space velocity, acceleration and jerk limits
   + Automatic stop after command time-out
   + Chainable Controller


Description of controller's interfaces
------------------------------------------------

References
,,,,,,,,,,,,,,,,,,

When controller is in chained mode, it exposes the following references which can be commanded by the preceding controller:

- ``<controller_name>/linear/velocity``      double, in m/s
- ``<controller_name>/angular/velocity``     double, in rad/s

Together, these represent the body twist (which in unchained-mode would be obtained from ~/cmd_vel).
The ``<controller_name>`` is commonly set to ``diff_drive_controller``.

Feedback
,,,,,,,,,,,,,,

As feedback interface type the joints' position (``hardware_interface::HW_IF_POSITION``) or velocity (``hardware_interface::HW_IF_VELOCITY``,if parameter ``position_feedback=false``) are used.
Unless the parameter ``open_loop=true`` is set, then no external state interfaces are used (the commanded velocity is used for odometry instead).

Output
,,,,,,,,,

Joints' velocity (``hardware_interface::HW_IF_VELOCITY``) are used.


ROS 2 Interfaces
------------------------

Subscribers
,,,,,,,,,,,,

~/cmd_vel [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.


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

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/src/diff_drive_controller_parameter.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/diff_drive_controller_parameter.yaml
  parameters_context.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/diff_drive_controller/test/config/test_diff_drive_controller.yaml>`_:

.. literalinclude:: ../test/config/test_diff_drive_controller.yaml
   :language: yaml
