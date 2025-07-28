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

pid_controller
*****************************
* Parameters ``enable_feedforward`` and service ``set_feedforward_control`` are removed. Instead, set the feedforward_gain to zero or a non-zero value. (`#1553 <https://github.com/ros-controls/ros2_controllers/pull/1553>`_).
* The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have
  been deprecated in favor of the new ``antiwindup_strategy`` parameter (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__). Choose a suitable anti-windup strategy and set the parameters accordingly.
* PID state publisher topic changed to ``<controller_name>`` namespace and is initially turned off. It can be turned on by using  ``activate_state_publisher`` parameter. (`#1823 <https://github.com/ros-controls/ros2_controllers/pull/1823>`_).
