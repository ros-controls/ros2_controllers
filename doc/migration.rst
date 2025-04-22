:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.

diff_drive_controller
*****************************
* Parameters ``has_velocity_limits``, ``has_acceleration_limits``, and ``has_jerk_limits`` are removed. Instead, set the respective limits to ``.NAN``. (`#1653 <https://github.com/ros-controls/ros2_controllers/pull/1653>`_).
