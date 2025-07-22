:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.


diff_drive_controller
*****************************
* The twist message on ``~/cmd_vel`` is now required to be of stamped type (`#812 <https://github.com/ros-controls/ros2_controllers/pull/812>`_).
* Parameters ``has_velocity_limits``, ``has_acceleration_limits``, and ``has_jerk_limits`` are removed. Instead, set the respective limits to ``.NAN``. (`#1315 <https://github.com/ros-controls/ros2_controllers/pull/1315>`_).

joint_trajectory_controller
*****************************
* Parameter ``allow_nonzero_velocity_at_trajectory_end`` is now per default ``false`` (`#834 <https://github.com/ros-controls/ros2_controllers/pull/834>`_).
* The parameter ``start_with_holding`` is removed, it now always holds the position at activation (`#839 <https://github.com/ros-controls/ros2_controllers/pull/839>`_).
* Goals are now cancelled in ``on_deactivate`` transition (`#962 <https://github.com/ros-controls/ros2_controllers/pull/962>`_).
* Empty trajectory messages are discarded (`#902 <https://github.com/ros-controls/ros2_controllers/pull/902>`_).
* Angle wraparound behavior (continuous joints) was added from the current state to the first segment of the incoming trajectory (`#796 <https://github.com/ros-controls/ros2_controllers/pull/796>`_).
* The URDF is now parsed for continuous joints and angle wraparound is automatically activated now (`#949 <https://github.com/ros-controls/ros2_controllers/pull/949>`_). Remove the ``angle_wraparound`` parameter from the configuration and set continuous joint type in the URDF of the respective joint.
* Tolerances sent with the action goal were not used before, but are now processed and used for the upcoming action. (`#716 <https://github.com/ros-controls/ros2_controllers/pull/716>`_). Adaptions to the action goal might be necessary.
* Parameter ``open_loop_control`` is replaced by ``interpolate_from_desired_state`` and setting the feedback gains to zero (`#1525 <https://github.com/ros-controls/ros2_controllers/pull/1525>`_).

pid_controller
************************
* Parameters ``enable_feedforward`` and service ``set_feedforward_control`` are deprecated. Instead, set the feedforward_gain to zero or a non-zero value. (`#1553 <https://github.com/ros-controls/ros2_controllers/pull/1553>`_).
* The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have
  been deprecated in favor of the new ``antiwindup_strategy`` parameter (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__). Choose a suitable anti-windup strategy and set the parameters accordingly.
<<<<<<< HEAD

steering_controllers_library
********************************
* ``front_steering`` parameter was removed (`#1166 <https://github.com/ros-controls/ros2_controllers/pull/1166>`_). Now every kinematics type (bicycle, tricycle, Ackermann) has dedicated parameters for steering or traction wheels instead of front/rear wheels.

  * *General*: Remove ``front_steering`` and use ``traction_joints_names``/``steering_joints_names`` instead of ``rear_wheels_names``/``front_wheels_names``, respectively.
  * *bicycle_steering_controller*: Set ``traction_wheel_radius`` instead of ``front_wheel_radius``, ``rear_wheel_radius``.
  * *tricycle_steering_controller*: Set ``traction_wheels_radius`` instead of ``front_wheels_radius``, ``rear_wheels_radius``.
  * *ackermann_steering_controller*: Set ``traction_wheels_radius`` instead of ``front_wheels_radius``, ``rear_wheels_radius``, and ``traction_track_width`` (and optionally ``steering_track_width``, if it differs) instead of ``rear_wheel_track``, ``front_wheel_track``.
=======
* PID state publisher topic changed to ``<controller_name>`` namespace and is initially turned off. It can be turned on by using  ``activate_state_publisher`` parameter. (`#1823 <https://github.com/ros-controls/ros2_controllers/pull/1823>`_).
>>>>>>> 0f51de6 (Apply API change of PidROS (#1823))
