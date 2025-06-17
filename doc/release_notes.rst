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
