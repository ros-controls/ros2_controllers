:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Humble (previous) and Jazzy (current) releases.

admittance_controller
************************
* Remove ``robot_description`` parameter from parameter YAML, because it is not used at all (`#963 <https://github.com/ros-controls/ros2_controllers/pull/963>`_).
* Added ``~/wrench_reference`` input topic which allows to provide a force-torque offset as WrenchStamped (`#1249 <https://github.com/ros-controls/ros2_controllers/pull/1249>`_).

diff_drive_controller
*****************************
* The twist message on ``~/cmd_vel`` is now required to be of stamped type (`#812 <https://github.com/ros-controls/ros2_controllers/pull/812>`_).
* Remove unused parameter ``wheels_per_side`` (`#958 <https://github.com/ros-controls/ros2_controllers/pull/958>`_).
* Parameters ``has_velocity_limits``, ``has_acceleration_limits``, and ``has_jerk_limits`` are removed. Instead, set the respective limits to ``.NAN``. (`#1315 <https://github.com/ros-controls/ros2_controllers/pull/1315>`_).
* Parameters ``max_acceleration_reverse`` and ``max_deceleration_reverse`` were added to configure asymmetric acceleration/deceleration behavior. (`#1315 <https://github.com/ros-controls/ros2_controllers/pull/1315>`_).

gripper_action_controller
*****************************
The ``effort_controllers/GripperActionController`` and ``position_controllers/GripperActionController`` will be removed. The ``parallel_gripper_action_controller/GripperActionController`` should be used instead. `(#1652 <https://github.com/ros-controls/ros2_controllers/pull/1652>`__).

chained_filter_controller
*******************************
* The chained_filter_controller was added to use generic filter plugins (`#1634 <https://github.com/ros-controls/ros2_controllers/pull/1634>`__).

joint_state_broadcaster
************************
* Make all parameters read-only (the never got re-evaluated after initialization anyways). (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)
* Added parameter ``publish_dynamic_joint_states`` to enable/disable publishing of dynamic joint states. (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)

joint_trajectory_controller
*****************************

* Parameter ``allow_nonzero_velocity_at_trajectory_end`` is now per default ``false`` (`#834 <https://github.com/ros-controls/ros2_controllers/pull/834>`_).
* Activate update of dynamic parameters (`#761 <https://github.com/ros-controls/ros2_controllers/pull/761>`_ and `#849 <https://github.com/ros-controls/ros2_controllers/pull/849>`_).
* The parameter ``start_with_holding`` is removed, it now always holds the position at activation (`#839 <https://github.com/ros-controls/ros2_controllers/pull/839>`_).
* Continue with last trajectory-point on success, instead of hold-position from current state (`#842 <https://github.com/ros-controls/ros2_controllers/pull/842>`_).
* Add console output for tolerance checks (`#932 <https://github.com/ros-controls/ros2_controllers/pull/932>`_):

  .. code::

    [tolerances]: State tolerances failed for joint 2:
    [tolerances]: Position Error: 0.020046, Position Tolerance: 0.010000
    [trajectory_controllers]: Aborted due goal_time_tolerance exceeding by 1.010000 seconds

* Goals are now cancelled in ``on_deactivate`` transition (`#962 <https://github.com/ros-controls/ros2_controllers/pull/962>`_).
* Empty trajectory messages are discarded (`#902 <https://github.com/ros-controls/ros2_controllers/pull/902>`_).
* Action field ``error_string`` is now filled with meaningful strings (`#887 <https://github.com/ros-controls/ros2_controllers/pull/887>`_).
* Angle wraparound behavior (continuous joints) was added from the current state to the first segment of the incoming trajectory (`#796 <https://github.com/ros-controls/ros2_controllers/pull/796>`_).
* The URDF is now parsed for continuous joints and angle wraparound is automatically activated now (`#949 <https://github.com/ros-controls/ros2_controllers/pull/949>`_). ``angle_wraparound`` parameter was completely removed.
* Tolerances sent with the action goal are now processed and used for the action. (`#716 <https://github.com/ros-controls/ros2_controllers/pull/716>`_). For details, see the `JointTolerance message <https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/JointTolerance.msg>`_:

  .. code-block:: markdown

    The tolerances specify the amount the position, velocity, and
    accelerations can vary from the setpoints.  For example, in the case
    of trajectory control, when the actual position varies beyond
    (desired position + position tolerance), the trajectory goal may
    abort.

    There are two special values for tolerances:
     * 0 - The tolerance is unspecified and will remain at whatever the default is
     * -1 - The tolerance is "erased".  If there was a default, the joint will be
            allowed to move without restriction.

* Add the boolean parameter ``set_last_command_interface_value_as_state_on_activation``. When set to ``true``, the last command interface value is used as both the current state and the last commanded state upon activation. When set to ``false``, the current state is used for both (`#1231 <https://github.com/ros-controls/ros2_controllers/pull/1231>`_).
* Feed-forward effort trajectories are supported now (`#1200 <https://github.com/ros-controls/ros2_controllers/pull/1200>`_).
* Parameter ``open_loop_control`` is replaced by ``interpolate_from_desired_state`` and setting the feedback gains to zero (`#1525 <https://github.com/ros-controls/ros2_controllers/pull/1525>`_).
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior. (`#1759 <https://github.com/ros-controls/ros2_controllers/pull/1759>`__).
* Fill in 0 velocities and accelerations into point before trajectories if the state interfaces
  don't contain velocity / acceleration information, but the trajectory does. This way, the segment
  up to the first waypoint will use the same interpolation as the rest of the trajectory. (`#2043
  <https://github.com/ros-controls/ros2_controllers/pull/2043>`_)

mecanum_drive_controller
************************
* 🚀 The mecanum_drive_controller was added 🎉 (`#512 <https://github.com/ros-controls/ros2_controllers/pull/512>`_).

parallel_gripper_action_controller
**********************************
The ``parallel_gripper_action_controller/GripperActionController`` was added. `(#1652 <https://github.com/ros-controls/ros2_controllers/pull/1652>`__).

omni_wheel_drive_controller
*********************************
* 🚀 The omni_wheel_drive_controller was added 🎉 (`#1535 <https://github.com/ros-controls/ros2_controllers/pull/1535>`_).

pid_controller
************************
* 🚀 The PID controller was added 🎉 (`#434 <https://github.com/ros-controls/ros2_controllers/pull/434>`_).
* Add ``save_i_term`` parameter to control retention of integral state after re-activation (`#1507 <https://github.com/ros-controls/ros2_controllers/pull/1507>`_).
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__).
  * Output clamping via ``u_clamp_max`` and ``u_clamp_min`` was added, allowing users to bound the controller output.
  * The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have been deprecated in favor of the new ``antiwindup_strategy`` parameter. A ``tracking_time_constant`` parameter has also been introduced to configure the back-calculation strategy.
  * A new ``error_deadband`` parameter stops integration when the error is within a specified range.
* PID state publisher can be turned off or on by using  ``activate_state_publisher`` parameter. (`#1823 <https://github.com/ros-controls/ros2_controllers/pull/1823>`_).

steering_controllers_library
********************************
* Changing default int values to double in steering controller's yaml file. The controllers should now initialize successfully without specifying these parameters (`#927 <https://github.com/ros-controls/ros2_controllers/pull/927>`_).
* A fix for Ackermann steering odometry was added (`#921 <https://github.com/ros-controls/ros2_controllers/pull/921>`_).
* Do not reset the steering wheels to ``0.0`` on timeout (`#1289 <https://github.com/ros-controls/ros2_controllers/pull/1289>`_).
* New parameter ``reduce_wheel_speed_until_steering_reached`` was added. If set to true, then the wheel speed(s) is reduced until the steering angle has been reached. Only considered if ``open_loop = false`` (`#1314 <https://github.com/ros-controls/ros2_controllers/pull/1314>`_).
* Ackermann kinematics now supports different wheel tracks for traction and steering axle (`#1166 <https://github.com/ros-controls/ros2_controllers/pull/1166>`_).
* ``front_steering`` parameter was removed, see migration notes (`#1166 <https://github.com/ros-controls/ros2_controllers/pull/1166>`_).

tricycle_controller
************************
* tricycle_controller now uses generate_parameter_library (`#957 <https://github.com/ros-controls/ros2_controllers/pull/957>`_).

gpio_controllers
************************
* The GPIO command controller was added 🎉 (`#1251 <https://github.com/ros-controls/ros2_controllers/pull/1251>`_).

force_torque_sensor_broadcaster
*******************************
* Multiplier support was added. Users can now specify per–axis scaling factors for both force and torque readings, applied after the existing offset logic. (`#1647 <https://github.com/ros-controls/ros2_controllers/pull/1647/files>`__).
* Added support for filter chains, allowing users to configure a sequence of filter plugins with their parameters. The force/torque sensor readings are filtered sequentially and published on a separate topic.
* Added support for transforming Wrench messages to a given list of target frames. This is useful when applications need force/torque data in their preferred coordinate frames. (`#2021 <https://github.com/ros-controls/ros2_controllers/pull/2021/files>`__).
