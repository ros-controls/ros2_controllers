^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.26.2 (2024-08-22)
-------------------

3.26.1 (2024-08-14)
-------------------

3.26.0 (2024-07-24)
-------------------
* Fix WaitSet issue in tests  (backport `#1206 <https://github.com/ros-controls/ros2_controllers/issues/1206>`_) (`#1212 <https://github.com/ros-controls/ros2_controllers/issues/1212>`_)
* Contributors: mergify[bot]

3.25.0 (2024-07-09)
-------------------

3.24.0 (2024-05-14)
-------------------

3.23.0 (2024-04-30)
-------------------

3.22.0 (2024-02-12)
-------------------
* Add test_depend on `hardware_interface_testing` (backport `#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_) (`#1020 <https://github.com/ros-controls/ros2_controllers/issues/1020>`_)
* Add tests for `interface_configuration_type` consistently (backport `#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_) (`#1007 <https://github.com/ros-controls/ros2_controllers/issues/1007>`_)
* Let sphinx add parameter description with nested structures to documentation (`#652 <https://github.com/ros-controls/ros2_controllers/issues/652>`_) (`#1006 <https://github.com/ros-controls/ros2_controllers/issues/1006>`_)
* Contributors: mergify[bot]

3.21.0 (2024-01-20)
-------------------

3.20.2 (2024-01-11)
-------------------

3.20.1 (2024-01-08)
-------------------

3.20.0 (2024-01-03)
-------------------

3.19.2 (2023-12-12)
-------------------

3.19.1 (2023-12-05)
-------------------

3.19.0 (2023-12-01)
-------------------
* Increase test coverage of interface configuration getters (backport `#856 <https://github.com/ros-controls/ros2_controllers/issues/856>`_)
* joint_state_broadcaster: Add proper subscription to TestCustomInterfaceMappingUpdate (`#859 <https://github.com/ros-controls/ros2_controllers/issues/859>`_) (`#864 <https://github.com/ros-controls/ros2_controllers/issues/864>`_)
* Contributors: Christoph Froehlich, mergify[bot]

3.18.0 (2023-11-21)
-------------------

3.17.0 (2023-10-31)
-------------------

3.16.0 (2023-09-20)
-------------------

3.15.0 (2023-09-11)
-------------------

3.14.0 (2023-08-16)
-------------------

3.13.0 (2023-08-04)
-------------------

3.12.0 (2023-07-18)
-------------------

3.11.0 (2023-06-24)
-------------------
* Added -Wconversion flag and fix warnings (`#667 <https://github.com/ros-controls/ros2_controllers/issues/667>`_)
* Contributors: gwalck

3.10.1 (2023-06-06)
-------------------

3.10.0 (2023-06-04)
-------------------

3.9.0 (2023-05-28)
------------------
* Use branch name substitution for all links (`#618 <https://github.com/ros-controls/ros2_controllers/issues/618>`_)
* [JTC] Fix deprecated header (`#610 <https://github.com/ros-controls/ros2_controllers/issues/610>`_)
* Fix github links on control.ros.org (`#604 <https://github.com/ros-controls/ros2_controllers/issues/604>`_)
* Contributors: Christoph Fröhlich

3.8.0 (2023-05-14)
------------------

3.7.0 (2023-05-02)
------------------

3.6.0 (2023-04-29)
------------------
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_)
* Fix docs format (`#589 <https://github.com/ros-controls/ros2_controllers/issues/589>`_)
* Contributors: Bence Magyar, Christoph Fröhlich

3.5.0 (2023-04-14)
------------------

3.4.0 (2023-04-02)
------------------

3.3.0 (2023-03-07)
------------------
* Add comments about auto-generated header files (`#539 <https://github.com/ros-controls/ros2_controllers/issues/539>`_)
* Contributors: AndyZe

3.2.0 (2023-02-10)
------------------
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_)
* Contributors: Tyler Weaver, Chris Thrasher

3.1.0 (2023-01-26)
------------------

3.0.0 (2023-01-19)
------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_)
* Contributors: Bence Magyar

2.15.0 (2022-12-06)
-------------------

2.14.0 (2022-11-18)
-------------------

2.13.0 (2022-10-05)
-------------------
* Generate parameters for Joint State Broadcaster (`#401 <https://github.com/ros-controls/ros2_controllers/issues/401>`_)
* Fix undeclared and wrong parameters in controllers. (`#438 <https://github.com/ros-controls/ros2_controllers/issues/438>`_)
  * Add missing parameter declaration in the joint state broadcaster.
  * Fix unsensible test in IMU Sensor Broadcaster.
* [JointStateBroadcaster] Reset internal variables to avoid duplication of joints (`#431 <https://github.com/ros-controls/ros2_controllers/issues/431>`_)
* Contributors: Denis Štogl, Gilmar Correia, Tyler Weaver, Bence Magyar

2.12.0 (2022-09-01)
-------------------
* Fix formatting CI job (`#418 <https://github.com/ros-controls/ros2_controllers/issues/418>`_)
* Contributors: Tyler Weaver

2.11.0 (2022-08-04)
-------------------
* Use explicit type in joint_state_broadcaster test (`#403 <https://github.com/ros-controls/ros2_controllers/issues/403>`_)
  This use of `auto` is causing a static assert on RHEL. Explicitly
  specifying the type seems to resolve the failure and allow the test to
  be compiled.
* Contributors: Scott K Logan

2.10.0 (2022-08-01)
-------------------

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
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Jafar Abdi

2.5.0 (2022-05-13)
------------------
* fix: :bug: make force_torque_sensor_broadcaster wait for realtime_publisher (`#327 <https://github.com/ros-controls/ros2_controllers/issues/327>`_)
* Contributors: Jaron Lundwall, Denis Štogl

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
* joint_state_broadcaster to use realtime tools (`#276 <https://github.com/ros-controls/ros2_controllers/issues/276>`_)
* Contributors: Bence Magyar

2.0.1 (2022-02-01)
------------------

2.0.0 (2022-01-28)
------------------

1.3.0 (2022-01-11)
------------------

1.2.0 (2021-12-29)
------------------
* [Joint State Broadcaster] Add mapping of custom states to standard values in "/joint_state" message (`#217 <https://github.com/ros-controls/ros2_controllers/issues/217>`_)
* [Joint State Broadcaster] Add option to support only specific interfaces on specific joints (`#216 <https://github.com/ros-controls/ros2_controllers/issues/216>`_)
* Contributors: Denis Štogl, Bence Magyar

1.1.0 (2021-10-25)
------------------
* Revise for-loop style (`#254 <https://github.com/ros-controls/ros2_controllers/issues/254>`_)
* Contributors: bailaC

1.0.0 (2021-09-29)
------------------
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* Unify style of controllers. (`#236 <https://github.com/ros-controls/ros2_controllers/issues/236>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* Contributors: Bence Magyar, Denis Štogl, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* [Joint State Broadcaster] Add option to publish joint states to local topics (`#218 <https://github.com/ros-controls/ros2_controllers/issues/218>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Reduce docs warnings and correct adding guidelines (`#219 <https://github.com/ros-controls/ros2_controllers/issues/219>`_)
* Contributors: Bence Magyar, Denis Štogl, Lovro Ivanov

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Add  rclcpp::shutdown(); to all standalone test functions
* Contributors: Denis Štogl

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------
* Remove unused variable (`#181 <https://github.com/ros-controls/ros2_controllers/issues/181>`_)
* Add extra joints parameter at joint state broadcaster (`#179 <https://github.com/ros-controls/ros2_controllers/issues/179>`_)
* Contributors: Cesc Folch Aldehuelo, Karsten Knese

0.2.1 (2021-05-03)
------------------
* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Rename joint_state_controller -> joint_state_broadcaster (`#160 <https://github.com/ros-controls/ros2_controllers/issues/160>`_)
  * Rename joint_state_controller -> _broadcaster
  * Update accompanying files (Ament, CMake, etc)
  * Update C++ from _controller to _broadcaster
  * Apply cpplint
  * Create stub controller to redirect to _broadcaster
  * Add test for loading old joint_state_controller
  * Add missing dependency on hardware_interface
  * Add link to documentation
  * Add joint_state_broadcaster to metapackage
  * Apply suggestions from code review
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update joint_state_broadcaster/joint_state_plugin.xml
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* Contributors: Bence Magyar, Matt Reynolds

* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Rename joint_state_controller -> joint_state_broadcaster (`#160 <https://github.com/ros-controls/ros2_controllers/issues/160>`_)
  * Rename joint_state_controller -> _broadcaster
  * Update accompanying files (Ament, CMake, etc)
  * Update C++ from _controller to _broadcaster
  * Apply cpplint
  * Create stub controller to redirect to _broadcaster
  * Add test for loading old joint_state_controller
  * Add missing dependency on hardware_interface
  * Add link to documentation
  * Add joint_state_broadcaster to metapackage
  * Apply suggestions from code review
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update joint_state_broadcaster/joint_state_plugin.xml
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* Contributors: Bence Magyar, Matt Reynolds

0.2.0 (2021-02-06)
------------------

0.1.2 (2021-01-07)
------------------

0.1.1 (2021-01-06)
------------------

0.1.0 (2020-12-23)
------------------
