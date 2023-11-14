:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/doc/parameters.rst

.. _parameters:

Details about parameters
^^^^^^^^^^^^^^^^^^^^^^^^

joints (list(string))
  Joint names to control and listen to.

command_joints (list(string))
  Joint names to control. This parameters is used if JTC is used in a controller chain where command and state interfaces don't have same names.

command_interface (list(string))
  Command interfaces provided by the hardware interface for all joints.

  Values: [position | velocity | acceleration] (multiple allowed)

state_interfaces (list(string))
  State interfaces provided by the hardware for all joints.

  Values: position (mandatory) [velocity, [acceleration]].
  Acceleration interface can only be used in combination with position and velocity.

action_monitor_rate (double)
  Rate to monitor status changes when the controller is executing action (control_msgs::action::FollowJointTrajectory).

  Default: 20.0

allow_partial_joints_goal (boolean)
  Allow joint goals defining trajectory for only some joints.

  Default: false

allow_integration_in_goal_trajectories (boolean)
  Allow integration in goal trajectories to accept goals without position or velocity specified

  Default: false

interpolation_method (string)
  The type of interpolation to use, if any. Can be "splines" or "none".

  Default: splines

open_loop_control (boolean)
  Use controller in open-loop control mode:

  * The controller ignores the states provided by hardware interface but using last commands as states for starting the trajectory interpolation.
  * It deactivates the feedback control, see the ``gains`` structure.

  This is useful if hardware states are not following commands, i.e., an offset between those (typical for hydraulic manipulators).

  .. Note::
     If this flag is set, the controller tries to read the values from the command interfaces on activation.
     If they have real numeric values, those will be used instead of state interfaces.
     Therefore it is important set command interfaces to NaN (i.e., ``std::numeric_limits<double>::quiet_NaN()``) or state values when the hardware is started.

  Default: false

start_with_holding (bool)
  If true, start with holding position after activation. Otherwise, no command will be sent until
  the first trajectory is received.

  Default: true

allow_nonzero_velocity_at_trajectory_end (boolean)
  If false, the last velocity point has to be zero or the goal will be rejected.

  Default: false

cmd_timeout (double)
  Timeout after which the input command is considered stale.
  Timeout is counted from the end of the trajectory (the last point).
  ``cmd_timeout`` must be greater than ``constraints.goal_time``,
  otherwise ignored.

  If zero, timeout is deactivated"

  Default: 0.0

constraints (structure)
  Default values for tolerances if no explicit values are states in JointTrajectory message.

constraints.stopped_velocity_tolerance (double)
  Default value for end velocity deviation.

  Default: 0.01

constraints.goal_time (double)
  Maximally allowed tolerance for not reaching the end of the trajectory in a predefined time.
  If set to zero, the controller will wait a potentially infinite amount of time.

  Default: 0.0 (not checked)

constraints.<joint_name>.trajectory (double)
  Maximally allowed deviation from the target trajectory for a given joint.

  Default: 0.0 (tolerance is not enforced)

constraints.<joint_name>.goal (double)
  Maximally allowed deviation from the goal (end of the trajectory) for a given joint.

  Default: 0.0 (tolerance is not enforced)

gains (structure)
  Only relevant, if ``open_loop_control`` is not set.

  If ``velocity`` is the only command interface for all joints or an ``effort`` command interface is configured, PID controllers are used for every joint.
  This structure contains the controller gains for every joint with the control law

  .. math::

     u = k_{ff} v_d + k_p e + k_i \sum e dt + k_d (v_d - v)

  with the desired velocity :math:`v_d`, the measured velocity :math:`v`, the position error :math:`e` (definition see ``angle_wraparound`` below),
  the controller period :math:`dt`, and the ``velocity`` or ``effort`` manipulated variable (control variable) :math:`u`, respectively.

gains.<joint_name>.p (double)
  Proportional gain :math:`k_p` for PID

  Default: 0.0

gains.<joint_name>.i (double)
  Integral gain :math:`k_i` for PID

  Default: 0.0

gains.<joint_name>.d (double)
  Derivative gain :math:`k_d` for PID

  Default: 0.0

gains.<joint_name>.i_clamp (double)
  Integral clamp. Symmetrical in both positive and negative direction.

  Default: 0.0

gains.<joint_name>.ff_velocity_scale (double)
  Feed-forward scaling :math:`k_{ff}` of velocity

  Default: 0.0

gains.<joint_name>.angle_wraparound (bool)
  For joints that wrap around (without end stop, ie. are continuous),
  where the shortest rotation to the target position is the desired motion.
  If true, the position error :math:`e = normalize(s_d - s)` is normalized between :math:`-\pi, \pi`.
  Otherwise  :math:`e = s_d - s` is used, with the desired position :math:`s_d` and the measured
  position :math:`s` from the state interface.

  Default: false
