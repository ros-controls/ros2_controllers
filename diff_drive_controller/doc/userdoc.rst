.. _diff_drive_controller_userdoc:

diff_drive_controller
---------------------

Controller for differential drive wheel systems. Control is in the form of a velocity command, that is split then sent on the two wheels of a differential drive wheel base. Odometry is computed from the feedback from the hardware, and published.

Velocity commands
^^^^^^^^^^^^^^^^^

The controller works with a velocity twist from which it extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Hardware interface type
^^^^^^^^^^^^^^^^^^^^^^^

The controller works with wheel joints through a velocity interface.

Other features
^^^^^^^^^^^^^^

    Realtime-safe implementation.
    Odometry publishing
    Task-space velocity, acceleration and jerk limits
    Automatic stop after command time-out
