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
