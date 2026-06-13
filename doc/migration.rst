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

tricycle_steering_controller
*****************************
* Use tricycle_steering_controller instead of tricycle_controller for single-drive tricycle steering (`#1695 <https://github.com/ros-controls/ros2_controllers/pull/1695>`_).
  Necessary changes can be found `here <https://github.com/ros-controls/gz_ros2_control/pull/865/changes>`_ and are summarized in the following:

  * ROS parameters

  .. code:: yaml

    # parameters to be changed from tricycle_controller
    traction_joint_name: traction_joint
    steering_joint_name: steering_joint
    wheel_radius: 0.3
    cmd_vel_timeout: 500

    # to tricycle_steering_controller
    traction_joints_names: ["traction_joint"]
    steering_joints_names: ["steering_joint"]
    traction_wheels_radius: 0.3
    reference_timeout: 0.5 # In seconds.
    reduce_wheel_speed_until_steering_reached: true # is default false

    # remove parameters
    odom_only_twist: false
    publish_ackermann_command: true

  * ROS command topic changed from ``/tricycle_controller/cmd_vel`` to ``/tricycle_controller/reference``.
  * Limiters: While tricycle_controller had limiters of the command interfaces does tricycle_steering_controller instead have limiters for the reference velocity. Update the URDF to enforce command limits from the ressource_manager, and add ``limits.angular.z`` and ``limits.linear.x`` if necessary.

tricycle_controller
*****************************
See tricycle_steering_controller above.
