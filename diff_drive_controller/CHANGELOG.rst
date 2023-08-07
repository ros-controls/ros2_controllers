^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.24.0 (2023-08-07)
-------------------

2.23.0 (2023-06-23)
-------------------
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_) (`#677 <https://github.com/ros-controls/ros2_controllers/issues/677>`_)
* Contributors: Bence Magyar

2.22.0 (2023-06-14)
-------------------
* Use generate_parameter_library for all params (`#601 <https://github.com/ros-controls/ros2_controllers/issues/601>`_) (`#627 <https://github.com/ros-controls/ros2_controllers/issues/627>`_)
* Docs: Use branch name substitution for all links (backport `#618 <https://github.com/ros-controls/ros2_controllers/issues/618>`_) (`#633 <https://github.com/ros-controls/ros2_controllers/issues/633>`_)
* [Formatting] enable ReflowComments to also use ColumnLimit on comments   (`#628 <https://github.com/ros-controls/ros2_controllers/issues/628>`_)
* Contributors: Sai Kishor Kothakota, Christoph Fröhlich

2.21.0 (2023-05-28)
-------------------
* Fix compilation warnings (`#621 <https://github.com/ros-controls/ros2_controllers/issues/621>`_) (`#623 <https://github.com/ros-controls/ros2_controllers/issues/623>`_)
* Remove compile warnings. (`#519 <https://github.com/ros-controls/ros2_controllers/issues/519>`_) (`#620 <https://github.com/ros-controls/ros2_controllers/issues/620>`_)
* Fix github links on control.ros.org (`#604 <https://github.com/ros-controls/ros2_controllers/issues/604>`_) (`#617 <https://github.com/ros-controls/ros2_controllers/issues/617>`_)
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_) (`#605 <https://github.com/ros-controls/ros2_controllers/issues/605>`_)
* Contributors: Felix Exner (fexner), Christoph Fröhlich, Mathias Lüdtke, Noel Jiménez García

2.20.0 (2023-05-14)
-------------------
* Clear registered handles of DiffDriveController on deactivate (`#596 <https://github.com/ros-controls/ros2_controllers/issues/596>`_) (`#606 <https://github.com/ros-controls/ros2_controllers/issues/606>`_)
* Contributors: Noel Jiménez García

2.19.0 (2023-05-02)
-------------------
* Fix wrong publish timestamp initialization (`#585 <https://github.com/ros-controls/ros2_controllers/issues/585>`_) (`#593 <https://github.com/ros-controls/ros2_controllers/issues/593>`_)
* Contributors: mergify[bot]

2.18.0 (2023-04-29)
-------------------
* adjusted open_loop param description in diff_drive_controller_parameter.yaml (`#570 <https://github.com/ros-controls/ros2_controllers/issues/570>`_) (`#576 <https://github.com/ros-controls/ros2_controllers/issues/576>`_)
* Contributors: muritane

2.17.3 (2023-04-14)
-------------------

2.17.2 (2023-03-07)
-------------------

2.17.1 (2023-02-20)
-------------------
* [DiffDriveController] Fix prefixing of frame id with controller's namespace (`#522 <https://github.com/ros-controls/ros2_controllers/issues/522>`_)
* Contributors: Tim Verbelen

2.17.0 (2023-02-13)
-------------------

2.16.1 (2023-01-31)
-------------------

2.16.0 (2023-01-19)
-------------------
* diff_drive base_frame_id param (`#495 <https://github.com/ros-controls/ros2_controllers/issues/495>`_) (`#498 <https://github.com/ros-controls/ros2_controllers/issues/498>`_)
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_) (`#493 <https://github.com/ros-controls/ros2_controllers/issues/493>`_)
* Contributors: Bence Magyar, Jakub Delicat

2.15.0 (2022-12-06)
-------------------
* [DiffDriveController] Use generate parameter library (`#386 <https://github.com/ros-controls/ros2_controllers/issues/386>`_)
* [DiffDriveController] Change units of velocity feedback (`#452 <https://github.com/ros-controls/ros2_controllers/issues/452>`_)
* Contributors: Maciej Stępień, Paul Gesel, Denis Štogl, Bence Magyar

2.14.0 (2022-11-18)
-------------------
* Odom Topic & Frame Namespaces  (`#461 <https://github.com/ros-controls/ros2_controllers/issues/461>`_)
* Write detailed Diff-Drive-Controller documentation to make all the interfaces understandable. (`#371 <https://github.com/ros-controls/ros2_controllers/issues/371>`_)
* Contributors: Denis Štogl, sp-sophia-labs

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------
* Fix formatting CI job (`#418 <https://github.com/ros-controls/ros2_controllers/issues/418>`_)
* Contributors: Tyler Weaver

2.11.0 (2022-08-04)
-------------------

2.10.0 (2022-08-01)
-------------------
* Formatting changes from pre-commit (`#400 <https://github.com/ros-controls/ros2_controllers/issues/400>`_)
* Parameter loading fixup in diff_drive and gripper controllers (`#385 <https://github.com/ros-controls/ros2_controllers/issues/385>`_)
* Contributors: Andy Zelenak, Tyler Weaver

2.9.0 (2022-07-14)
------------------

2.8.0 (2022-07-09)
------------------

2.7.0 (2022-07-03)
------------------
* Update controllers with new get_name hardware interfaces (`#369 <https://github.com/ros-controls/ros2_controllers/issues/369>`_)
* Contributors: Lucas Schulze

2.6.0 (2022-06-18)
------------------
* Disable failing workflows (`#363 <https://github.com/ros-controls/ros2_controllers/issues/363>`_)
* CMakeLists cleanup (`#362 <https://github.com/ros-controls/ros2_controllers/issues/362>`_)
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Andy Zelenak, Jafar Abdi

2.5.0 (2022-05-13)
------------------
* [diff_drive_controller] Made odom topic name relative as it was in ROS1. (`#343 <https://github.com/ros-controls/ros2_controllers/issues/343>`_)
* Fix wrong integration of velocity feedback in odometry in diff_drive_controller (`#331 <https://github.com/ros-controls/ros2_controllers/issues/331>`_)
* Contributors: Patrick Roncagliolo, Tony Baltovski

2.4.0 (2022-04-29)
------------------
* updated to use node getter functions (`#329 <https://github.com/ros-controls/ros2_controllers/issues/329>`_)
* Contributors: Bence Magyar, Denis Štogl, Jack Center

2.3.0 (2022-04-21)
------------------
* Use CallbackReturn from controller_interface namespace (`#333 <https://github.com/ros-controls/ros2_controllers/issues/333>`_)
* Contributors: Bence Magyar, Denis Štogl

2.2.0 (2022-03-25)
------------------
* Use lifecycle node as base for controllers (`#244 <https://github.com/ros-controls/ros2_controllers/issues/244>`_)
* Contributors: Denis Štogl, Vatan Aksoy Tezer, Bence Magyar

2.1.0 (2022-02-23)
------------------
* use rolling mean from rcppmath (`#211 <https://github.com/ros-controls/ros2_controllers/issues/211>`_)
* Contributors: Karsten Knese, Bence Magyar

2.0.1 (2022-02-01)
------------------

2.0.0 (2022-01-28)
------------------

1.3.0 (2022-01-11)
------------------
* Add publish_rate option for the diff_drive_controller (`#278 <https://github.com/ros-controls/ros2_controllers/issues/278>`_)
* Fix angular velocity direction of diff_drive_controller odometry (`#281 <https://github.com/ros-controls/ros2_controllers/issues/281>`_)
* Contributors: Benjamin Hug, Paul Verhoeckx

1.2.0 (2021-12-29)
------------------
* Add velocity feedback option for diff_drive_controller (`#260 <https://github.com/ros-controls/ros2_controllers/issues/260>`_)
* Contributors: Patrick Roncagliolo

1.1.0 (2021-10-25)
------------------
* Use common test URDF from descriptions.hpp (`#258 <https://github.com/ros-controls/ros2_controllers/issues/258>`_)
* Fix header include on Fedora <https://github.com/ros-controls/ros2_controllers/issues/255>`_ (`#256 <https://github.com/ros-controls/ros2_controllers/issues/256>`_)
* Fix diff_drive accel limit (`#242 <https://github.com/ros-controls/ros2_controllers/issues/242>`_) (`#252 <https://github.com/ros-controls/ros2_controllers/issues/252>`_)
* Contributors: Denis Štogl, Josh Newans, Noeël Moeskops, bailaC

1.0.0 (2021-09-29)
------------------
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* Unify style of controllers. (`#236 <https://github.com/ros-controls/ros2_controllers/issues/236>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* refactor get_current_state to get_state (`#232 <https://github.com/ros-controls/ros2_controllers/issues/232>`_)
* Contributors: Bence Magyar, Denis Štogl, Márk Szitanics, bailaC

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
