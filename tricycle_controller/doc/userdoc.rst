:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/tricycle_controller/doc/userdoc.rst

.. _tricycle_controller_userdoc:

tricycle_controller
=====================

Controller for mobile robots with a single double-actuated wheel, including traction and steering. An example is a tricycle robot with the actuated wheel in the front and two trailing wheels on the rear axle.

Input for control are robot base_link twist commands which are translated to traction and steering
commands for the tricycle drive base. Odometry is computed from hardware feedback and published.

For an introduction to mobile robot kinematics and the nomenclature used here, see :ref:`mobile_robot_kinematics`.

Other features
--------------

    Realtime-safe implementation.
    Odometry publishing
    Velocity, acceleration and jerk limits
    Automatic stop after command timeout


ROS 2 Interfaces
------------------------

Subscribers
,,,,,,,,,,,,

~/cmd_vel [geometry_msgs/msg/TwistStamped]
  Velocity command for the controller. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.
