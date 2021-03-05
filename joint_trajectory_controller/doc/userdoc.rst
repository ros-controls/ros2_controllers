.. _joint_trajectory_controller_userdoc:

joint_trajectory_controller
---------------------------

Controller for executing joint-space trajectories on a group of joints. Trajectories are specified as a set of waypoints to be reached at specific time instants, which the controller attempts to execute as well as the mechanism allows. Waypoints consist of positions, and optionally velocities and accelerations.

Trajectory representation
^^^^^^^^^^^^^^^^^^^^^^^^^

The controller is templated to work with multiple trajectory representations. By default, a spline interpolator is provided, but it's possible to support other representations. The spline interpolator uses the following interpolation strategies depending on the waypoint specification:

    Linear: Only position is specified. Guarantees continuity at the position level. Discouraged because it yields trajectories with discontinuous velocities at the waypoints.

    Cubic: Position and velocity are specified. Guarantees continuity at the velocity level.

    Quintic: Position, velocity and acceleration are specified: Guarantees continuity at the acceleration level.

Hardware interface type
^^^^^^^^^^^^^^^^^^^^^^^

The controller is templated to work with multiple hardware interface types. Currently joints with position, velocity and effort interfaces are supported. For position-controlled joints, desired positions are simply forwarded to the joints; while for velocity (effort) joints, the position+velocity trajectory following error is mapped to velocity (effort) commands through a PID loop. Example controller configurations can be found here.

Similarly to the trajectory representation case above, it's possible to support new hardware interfaces, or alternative mappings to an already supported interface (eg. a proxy controller for generating effort commands).

Other features
^^^^^^^^^^^^^^

    Realtime-safe implementation.

    Proper handling of wrapping (continuous) joints.

    Robust to system clock changes: Discontinuous system clock changes do not cause discontinuities in the execution of already queued trajectory segments.