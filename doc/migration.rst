:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.

joint_state_broadcaster
*****************************
* Removed interfaces with other data types than double for publishing to ``dynamic_joint_states``. (`#2115 <https://github.com/ros-controls/ros2_controllers/pull/2115>`_).
  Use a custom controller for publishing non-double interfaces.
* Parameter ``publish_dynamic_joint_states`` is now deprecated (default changed to ``false``). (`#2107 <https://github.com/ros-controls/ros2_controllers/pull/2107>`_)
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

position_controllers
*****************************
* ``position_controllers/JointGroupPositionController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``position``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).

velocity_controllers
*****************************
*  ``velocity_controllers/JointGroupVelocityController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``velocity``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).

diff_drive_controller
*****************************
* Instead of using ``tf_frame_prefix_enable:=false``, set an empty ``tf_frame_prefix:=""`` parameter instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* For using node namespace as tf prefix: Set ``tf_frame_prefix:="~"``, where the ("~") character is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).

mecanum_drive_controller
*****************************
* Instead of using ``tf_frame_prefix_enable:=false``, set an empty ``tf_frame_prefix:=""`` parameter instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* For using node namespace as tf prefix: Set ``tf_frame_prefix:="~"``, where the ("~") character is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).

omni_wheel_drive_controller
*****************************
* Instead of using ``tf_frame_prefix_enable:=false``, set an empty ``tf_frame_prefix:=""`` parameter instead. (`#2073 <https://github.com/ros-controls/ros2_controllers/pull/2073>`_).
* For using node namespace as tf prefix: Set ``tf_frame_prefix:="~"``, where the ("~") character is substituted with node namespace. (`#2073 <https://github.com/ros-controls/ros2_controllers/pull/2073>`_).
