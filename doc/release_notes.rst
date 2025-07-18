:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

force_torque_sensor_broadcaster
*******************************
* Multiplier support was added. Users can now specify per–axis scaling factors for both force and torque readings, applied after the existing offset logic. (`#1647 <https://github.com/ros-controls/ros2_controllers/pull/1647/files>`__).

joint_trajectory_controller
*******************************
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior. (`#1759 <https://github.com/ros-controls/ros2_controllers/pull/1759>`__).

pid_controller
*******************************
* The controller now supports the new anti-windup strategy of the PID class, which allows for more flexible control of the anti-windup behavior (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__).
  * Output clamping via ``u_clamp_max`` and ``u_clamp_min`` was added, allowing users to bound the controller output.
  * The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have been deprecated in favor of the new ``antiwindup_strategy`` parameter. A ``tracking_time_constant`` parameter has also been introduced to configure the back-calculation strategy.
  * A new ``error_deadband`` parameter stops integration when the error is within a specified range.