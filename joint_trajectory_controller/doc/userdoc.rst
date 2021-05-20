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


Using Joint Trajectory Controller(s)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The controller expects at least position feedback from the hardware.
Joint velocities and accelerations are optional.
Currently the controller does not internally integrate velocity from acceleration and position from velocity.
Therefore if the hardware provides only acceleration or velocity states they have to be integrated in the hardware-interface implementation of velocity and position to use these controllers.

The generic version of Joint Trajectory controller is implemented in this package.
A yaml file for using it could be:

   .. code-block:: yaml

      controller_manager:
        ros__parameters:
          joint_trajectory_controller:
          type: "joint_trajectory_controller/JointTrajectoryController"

      joint_trajectory_controller:
        ros__parameters:
          joints:
            - joint0
            - joint1
            - joint2
            - joint3
            - joint4
            - joint5

          command_interfaces:
            - position

          state_interfaces:
            - position
            - velocity

          state_publish_rate: 50.0 # Defaults to 50
          action_monitor_rate: 20.0 # Defaults to 20

          allow_partial_joints_goal: false # Defaults to false
          hardware_state_has_offset: true
          deduce_states_from_derivatives: true
          constraints:
            stopped_velocity_tolerance: 0.01 # Defaults to 0.01
            goal_time: 0.0 # Defaults to 0.0 (start immediately)


Specialized versions of JointTrajectoryController (TBD in ...)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The controller types are placed into namespaces according to their command types for the hardware (see `general introduction into controllers <../../index.rst>`_).

The following version of the Joint Trajectory Controller are available mapping the following interfaces:

  - position_controllers::JointTrajectoryController
    - Input: position, [velocity, [acceleration]]
    - Output: position
  - position_velocity_controllers::JointTrajectoryController
    - Input: position, [velocity, [acceleration]]
    - Output: position and velocity
  - position_velocity_acceleration_controllers::JointTrajectoryController
    - Input: position, [velocity, [acceleration]]
    - Output: position, velocity and acceleration
..   - velocity_controllers::JointTrajectoryController
..     - Input: position, [velocity, [acceleration]]
..     - Output: velocity
.. TODO(anyone): would it be possible to output velocty and acceleration?
..               (to have an vel_acc_controllers)
..   - effort_controllers::JointTrajectoryController
..     - Input: position, [velocity, [acceleration]]
..     - Output: effort

The controller uses `common hardware interface definitions`_.

(*Not implemented yet*) When using pure ``velocity`` or ``effort`` controllers a command is generated using the desired state and state error using a velocity feedforward term plus a corrective PID term. (#171)
