:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.

diff_drive_controller
*****************************
* Instead of using ``tf_frame_prefix_enable:=false``, set an empty ``tf_frame_prefix:=""`` parameter instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* For using node namespace as tf prefix: Set ``tf_frame_prefix:="~"``, where the ("~") character is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
