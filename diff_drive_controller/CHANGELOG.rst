^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2022-05-31)
------------------

0.7.0 (2022-01-24)
------------------

0.6.0 (2022-01-11)
------------------
* Add publish_rate option for the diff_drive_controller (backport `#278 <https://github.com/ros-controls/ros2_controllers/issues/278>`_) (`#284 <https://github.com/ros-controls/ros2_controllers/issues/284>`_)
* Contributors: Benjamin Hug

0.5.1 (2021-10-25)
------------------
* Fix diff_drive accel limit (`#242 <https://github.com/ros-controls/ros2_controllers/issues/242>`_) (backport `#252 <https://github.com/ros-controls/ros2_controllers/issues/252>`_) (`#257 <https://github.com/ros-controls/ros2_controllers/issues/257>`_)
* Contributors: Josh Newans

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Reduce docs warnings and correct adding guidelines (`#219 <https://github.com/ros-controls/ros2_controllers/issues/219>`_)
* Contributors: Bence Magyar, Denis Štogl, Lovro Ivanov

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Add  rclcpp::shutdown(); to all standalone test functions
* Fixes for Windows (`#205 <https://github.com/ros-controls/ros2_controllers/issues/205>`_)
  * Fix MSVC build for diff_drive_controller test
* Fix parameter initialisation for galactic (`#199 <https://github.com/ros-controls/ros2_controllers/issues/199>`_)
* Contributors: Akash, Denis Štogl, Tim Clephas

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------

0.2.1 (2021-05-03)
------------------
* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Add basic user docs pages for each package (`#156 <https://github.com/ros-controls/ros2_controllers/issues/156>`_)
* [diff_drive_controller] Change header math.h in cmath for better C++ compliance (`#148 <https://github.com/ros-controls/ros2_controllers/issues/148>`_)
  and isnan inclusion.
* Contributors: Bence Magyar, Olivier Stasse

0.2.0 (2021-02-06)
------------------
* Fix diff drive twist concurrency issues (`#146 <https://github.com/ros-controls/ros2_controllers/issues/146>`_)
  * Fix diff drive twist concurrency issues
  Before this fix, a twist message could be received and stored one
  thread, in the middle of the update() of the controller.
  This would be fixed by making a copy of the shared pointer at the
  beginning of the update() function, added realtime box to ensure safe
  concurrent access to the pointer.
  * Don't store limited command as last command
  Before these changes, the limited command overwrote the original
  command, which mean that it too much more time to reach the commanded
  speed.
  We only want this behavior when the command is too old and we replace it
  with 0 speed.
* Diff drive parameter fixes (`#145 <https://github.com/ros-controls/ros2_controllers/issues/145>`_)
  * Recover old speed limiter behavior, if unspecified min defaults to -max
  * Change cmd_vel_timeout to seconds (double) as ROS1 instead of ms(int)
* Unstamped cmd_vel subscriber rebased (`#143 <https://github.com/ros-controls/ros2_controllers/issues/143>`_)
* Contributors: Anas Abou Allaban, Victor Lopez

0.1.2 (2021-01-07)
------------------
* Remove unused sensor_msgs dependency (was non-declared in package.xml) (`#139 <https://github.com/ros-controls/ros2_controllers/issues/139>`_)
* Contributors: Bence Magyar

0.1.1 (2021-01-06)
------------------
* avoid warnings (`#137 <https://github.com/ros-controls/ros2_controllers/issues/137>`_)
* Migrate diff drive controller to resourcemanager (`#128 <https://github.com/ros-controls/ros2_controllers/issues/128>`_)
* Contributors: Bence Magyar, Karsten Knese

0.1.0 (2020-12-23)
------------------
