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
