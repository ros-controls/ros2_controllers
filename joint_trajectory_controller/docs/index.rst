=================================
Joint Trajectory Controller(s)
=================================

The controller expects at least position feedback from the hardware.
Joint velocities and accelerations are optional.
Currently the controller does not internally integrate velocity from acceleration and position from velocity.
Therefore if the hardware provides only acceleration or velocity states they have to be integrated in the hardware-interface implementation ot velocitiy and position to use these controllers.

The controller types are placed into namespaces according to their command types for the hardware (see `general introduction into controllers <../../index.rst>`_).

The following version of the Joint Trajectory Controller are available mapping the following interfaces:

  - position_controllers::JointTrajectoryController
    - Input: position, [velocity, [acceleration]]
    - Output: position
  - position_velocity_controllers::JointTrajectoryController
    - Input: position, [velocity, [acceleration]]
    - Output: position and velocity
  - position_velocityy_acceleration_controllers::JointTrajectoryController
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

(*Not implemented yet*) When using a pure ``velocity`` or ``effort`` controllers a command is generated using the desired state and state error using a velocity feedforward term plus a corrective PID term.

A yaml file for joint trajectory controllers looks something like:

1. Position-control hardware interface:

   .. code-block:: yaml

     head_controller:
     type: "position_controllers/JointTrajectoryController"
     joints:
       - head_1_joint
       - head_2_joint

     constraints:
       goal_time: 0.6
       stopped_velocity_tolerance: 0.02
       head_1_joint: {trajectory: 0.05, goal: 0.02}
       head_2_joint: {trajectory: 0.05, goal: 0.02}
       stop_trajectory_duration: 0.5
       state_publish_rate:  25


2. Position-control hardware interface:

   .. code-block:: yaml
