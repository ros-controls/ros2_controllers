:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

<<<<<<< HEAD
force_torque_sensor_broadcaster
*******************************
* Added support for transforming Wrench messages to a given list of target frames. This is useful when applications need force/torque data in their preferred coordinate frames. (`#2021 <https://github.com/ros-controls/ros2_controllers/pull/2021/files>`__).
=======
diff_drive_controller
*****************************
* Remove unused parameter ``tf_frame_prefix_enable``, use ``tf_frame_prefix`` instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
>>>>>>> 55ac4411 (deprecated tf prefix flag)
