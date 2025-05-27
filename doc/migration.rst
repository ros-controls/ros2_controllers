:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.

GripperActionController
*****************************
The ``effort_controllers/GripperActionController`` and ``position_controllers/GripperActionController`` have been removed. The ``parallel_gripper_action_controller/GripperActionController`` should be used instead. `(#1652 <https://github.com/ros-controls/ros2_controllers/pull/1652>`__).

diff_drive_controller
*****************************
* Parameters ``has_velocity_limits``, ``has_acceleration_limits``, and ``has_jerk_limits`` are removed. Instead, set the respective limits to ``.NAN``. (`#1653 <https://github.com/ros-controls/ros2_controllers/pull/1653>`_).
