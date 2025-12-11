:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

admittance_controller
**********************
* Add filter capability (`#560 <https://github.com/ros-controls/ros2_controllers/pull/560>`__).

force_torque_sensor_broadcaster
*******************************
* Added support for transforming Wrench messages to a given list of target frames. This is useful when applications need force/torque data in their preferred coordinate frames. (`#2021 <https://github.com/ros-controls/ros2_controllers/pull/2021/files>`__).

diff_drive_controller
*****************************
* Parameter ``tf_frame_prefix_enable`` got deprecated and will be removed in a future release (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).

joint_state_broadcaster
************************
* Make all parameters read-only (the never got re-evaluated after initialization anyways). (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)
* Added parameter ``publish_dynamic_joint_states`` to enable/disable publishing of dynamic joint states. (`#2064 <https://github.com/ros-controls/ros2_controllers/pull/2064>`_)

joint_trajectory_controller
***************************
* Fill in 0 velocities and accelerations into point before trajectories if the state interfaces
  don't contain velocity / acceleration information, but the trajectory does. This way, the segment
  up to the first waypoint will use the same interpolation as the rest of the trajectory. (`#2043
  <https://github.com/ros-controls/ros2_controllers/pull/2043>`_)
