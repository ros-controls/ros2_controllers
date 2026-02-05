:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

state_interfaces_broadcaster
*********************************
* 🚀 The state_interfaces_broadcaster was added 🎉 (`#2006 <https://github.com/ros-controls/ros2_controllers/pull/2006>`_).

force_torque_sensor_broadcaster
*******************************
* Added support for transforming Wrench messages to a given list of target frames. This is useful when applications need force/torque data in their preferred coordinate frames. (`#2021 <https://github.com/ros-controls/ros2_controllers/pull/2021/files>`__).

diff_drive_controller
*****************************
* Parameter ``tf_frame_prefix_enable`` got deprecated and will be removed in a future release (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Set odometry service added to be used at runtime. (`#2096 <https://github.com/ros-controls/ros2_controllers/pull/2096>`_).

mecanum_drive_controller
*****************************
* Parameter ``tf_frame_prefix_enable`` got deprecated and will be removed in a future release (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).

joint_state_broadcaster
************************
* Make all parameters read-only (the never got re-evaluated after initialization anyways). (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)
* Added parameter ``publish_dynamic_joint_states`` to enable/disable publishing of dynamic joint states. (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)
* Removed interfaces with other data types than double for publishing to ``dynamic_joint_states``. (`#2115 <https://github.com/ros-controls/ros2_controllers/pull/2115>`_)

omni_wheel_drive_controller
*****************************
* Parameter ``tf_frame_prefix_enable`` got deprecated and will be removed in a future release (`#2073 <https://github.com/ros-controls/ros2_controllers/pull/2073>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#2073 <https://github.com/ros-controls/ros2_controllers/pull/2073>`_).

joint_trajectory_controller
*******************************
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior. (`#1759 <https://github.com/ros-controls/ros2_controllers/pull/1759>`__).
* Scaling support was added in `#1191
  <https://github.com/ros-controls/ros2_controllers/pull/1191>`__. With this the controller
  "stretches the time" with which it progresses in the trajectory. Scaling can either be set
  manually or it can be synchronized with the hardware. See :ref:`jtc_speed_scaling` for details.

omni_wheel_drive_controller
*********************************
* 🚀 The omni_wheel_drive_controller was added 🎉 (`#1535 <https://github.com/ros-controls/ros2_controllers/pull/1535>`_).

pid_controller
*******************************
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__).
  * Output clamping via ``u_clamp_max`` and ``u_clamp_min`` was added, allowing users to bound the controller output.
  * The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have been deprecated in favor of the new ``antiwindup_strategy`` parameter. A ``tracking_time_constant`` parameter has also been introduced to configure the back-calculation strategy.
  * A new ``error_deadband`` parameter stops integration when the error is within a specified range.
* PID state publisher can be turned off or on by using  ``activate_state_publisher`` parameter. (`#1823 <https://github.com/ros-controls/ros2_controllers/pull/1823>`_).

swerve_drive_controller
*********************************
* The swerve_drive_controller was added  (`#1694 <https://github.com/ros-controls/ros2_controllers/pull/1694>`_).
