:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.

GripperActionController
*****************************
<<<<<<< HEAD
The ``effort_controllers/GripperActionController`` and ``position_controllers/GripperActionController`` have been removed. The ``parallel_gripper_action_controller/GripperActionController`` should be used instead. `(#1652 <https://github.com/ros-controls/ros2_controllers/pull/1652>`__).
=======
* Removed interfaces with other data types than double for publishing to ``dynamic_joint_states``. (`#2115 <https://github.com/ros-controls/ros2_controllers/pull/2115>`_).
  Use a custom controller for publishing non-double interfaces.
* Parameter ``publish_dynamic_joint_states`` is removed. (`#2107 <https://github.com/ros-controls/ros2_controllers/pull/2107>`_, `#2187 <https://github.com/ros-controls/ros2_controllers/pull/2187>`_)
  For publishing non-standard interfaces, consider using alternatives:

  * :ref:`state_interfaces_broadcaster <state_interfaces_broadcaster_userdoc>` for broadcasting arbitrary state interfaces
  * :ref:`gpio_controllers <gpio_controllers_userdoc>` for GPIO and custom hardware interfaces
  * `pal_statistics <https://github.com/pal-robotics/pal_statistics>`_ for flexible runtime statistics publishing

effort_controllers
*****************************
* ``effort_controllers/JointGroupEffortController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``effort``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).

joint_trajectory_controller
*****************************
* The default value of the ``constraints.goal_time`` parameter changed from ``0.0`` to ``10.0`` seconds and its semantics changed.
  Previously ``goal_time`` of ``0.0`` meant that the controller would wait a potentially infinite amount of time for the goal to be reached.
  Now, any **negative** ``goal_time`` (e.g. ``-1.0``) means the controller will wait indefinitely, while ``0.0`` disables the grace period so the goal must be reached on time, and positive values set a finite grace period in seconds.
  Configurations that relied on the old default or on ``goal_time: 0.0`` for an infinite timeout must set ``constraints.goal_time`` to a negative value (e.g. ``-1.0``) explicitly.
  Note that ``cmd_timeout`` only activates when ``cmd_timeout > constraints.goal_time``; with the new default timeout feature setups relying on ``cmd_timeout`` (e.g. to hold position at the end of the trajectory) must set ``constraints.goal_time`` accordingly.
  The ``action_execution_timeout`` parameter is removed.
* With a position command interface, cancelling a goal now holds, or decelerates to, the last commanded position instead of the last measured one.
  Anchoring to the measured position stepped the command backward by the following error in a single control period, which reads downstream as a large, unintended deceleration.
* The deceleration ramp on cancel now ignores speed scaling: it starts from the joints' actual velocity and decelerates at ``max_deceleration_on_cancel`` in wall-clock time.

position_controllers
*****************************
* ``position_controllers/JointGroupPositionController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``position``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).

velocity_controllers
*****************************
*  ``velocity_controllers/JointGroupVelocityController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``velocity``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).
>>>>>>> dd12289 ([JTC] Anchor the cancel stop to the last command, and guard the hold against non-finite state (#2587))

diff_drive_controller
*****************************
* Parameters ``has_velocity_limits``, ``has_acceleration_limits``, and ``has_jerk_limits`` are removed. Instead, set the respective limits to ``.NAN``. (`#1653 <https://github.com/ros-controls/ros2_controllers/pull/1653>`_).

pid_controller
*****************************
* Parameters ``enable_feedforward`` and service ``set_feedforward_control`` are removed. Instead, set the feedforward_gain to zero or a non-zero value. (`#1553 <https://github.com/ros-controls/ros2_controllers/pull/1553>`_).
* The legacy ``antiwindup`` boolean and integral clamp parameters ``i_clamp_max``/``i_clamp_min`` have
  been deprecated in favor of the new ``antiwindup_strategy`` parameter (`#1585 <https://github.com/ros-controls/ros2_controllers/pull/1585>`__). Choose a suitable anti-windup strategy and set the parameters accordingly.
* PID state publisher topic changed to ``<controller_name>`` namespace and is initially turned off. It can be turned on by using  ``activate_state_publisher`` parameter. (`#1823 <https://github.com/ros-controls/ros2_controllers/pull/1823>`_).
