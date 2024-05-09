:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/tricycle_controller/doc/userdoc.rst

.. _tricycle_controller_userdoc:

tricycle_controller
=====================

Controller for mobile robots with a single double-actuated wheel, including traction and steering. An example is a tricycle robot with the actuated wheel in the front and two trailing wheels on the rear axle.

Input for control are robot base_link twist commands which are translated to traction and steering
commands for the tricycle drive base. Odometry is computed from hardware feedback and published.

Velocity commands
-----------------

The controller works with a velocity twist from which it extracts
the x component of the linear velocity and the z component of the angular velocity.
Velocities on other components are ignored.


Other features
--------------

    Realtime-safe implementation.
    Odometry publishing
    Velocity, acceleration and jerk limits
    Automatic stop after command timeout

Parameters
--------------

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

.. generate_parameter_library_details:: ../src/tricycle_controller_parameter.yaml
