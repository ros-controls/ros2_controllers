:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

chained_filter_controller
*******************************
* The chained_filter_controller was added to use generic filter plugins (`#1634 <https://github.com/ros-controls/ros2_controllers/pull/1634>`__).

force_torque_sensor_broadcaster
*******************************
* Multiplier support was added. Users can now specify per–axis scaling factors for both force and torque readings, applied after the existing offset logic. (`#1647 <https://github.com/ros-controls/ros2_controllers/pull/1647/files>`__).
* Added support for filter chains, allowing users to configure a sequence of filter plugins with their parameters. The force/torque sensor readings are filtered sequentially and published on a separate topic.

imu_sensor_broadcaster
*******************************
* IMU sensor broadcaster is now a chainable controller. It supports a calibration by means of a rotation, defined as euler angles, to its target frame. (`#1833 <https://github.com/ros-controls/ros2_controllers/pull/1833/files>`__).

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

motion_primitives_forward_controller
*******************************************
* 🚀 The motion_primitives_forward_controller was added 🎉 (`#1636 <https://github.com/ros-controls/ros2_controllers/pull/1636>`_).
