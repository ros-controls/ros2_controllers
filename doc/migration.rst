:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.

effort_controllers
*****************************
* ``effort_controllers/JointGroupEffortController`` is deprecated. Use :ref:`forward_command_controller <forward_command_controller_userdoc>` instead by adding the ``interface_name`` parameter and set it to ``effort``. (`#1913 <https://github.com/ros-controls/ros2_controllers/pull/1913>`_).

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

omni_wheel_drive_controller
*****************************
* Instead of using ``tf_frame_prefix_enable:=false``, set an empty ``tf_frame_prefix:=""`` parameter instead. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
* For using node namespace as tf prefix: Set ``tf_frame_prefix:="~"``, where the ("~") character is substituted with node namespace. (`#1997 <https://github.com/ros-controls/ros2_controllers/pull/1997>`_).
