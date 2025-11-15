:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.

diff_drive_controller
*****************************
* Remove unused parameter ``tf_frame_prefix_enable``, use ``tf_frame_prefix`` instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* Now any tilde ("~") character in ``tf_frame_prefix`` is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
