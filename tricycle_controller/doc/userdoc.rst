.. _tricycle_controller_userdoc:

tricycle_controller
=====================

Controller for mobile robots with tricycle drive.
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