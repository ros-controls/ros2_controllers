:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Iron (previous) and Jazzy (current) releases, where changes to user code might be necessary.


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

steering_controllers_library
********************************
* ``front_steering`` parameter was removed (`#1166 <https://github.com/ros-controls/ros2_controllers/pull/1166>`_). Now every kinematics type (bicycle, tricycle, Ackermann) has dedicated parameters for steering or traction wheels instead of front/rear wheels.

  * *General*: Remove ``front_steering`` and use ``traction_joints_names``/``steering_joints_names`` instead of ``rear_wheels_names``/``front_wheels_names``, respectively.
  * *bicycle_steering_controller*: Set ``traction_wheel_radius`` instead of ``front_wheel_radius``, ``rear_wheel_radius``.
  * *tricycle_steering_controller*: Set ``traction_wheels_radius`` instead of ``front_wheels_radius``, ``rear_wheels_radius``.
  * *ackermann_steering_controller*: Set ``traction_wheels_radius`` instead of ``front_wheels_radius``, ``rear_wheels_radius``, and ``traction_wheel_track`` (and optionally ``steering_wheel_track``, if it differs) instead of ``rear_wheel_track``, ``front_wheel_track``.
