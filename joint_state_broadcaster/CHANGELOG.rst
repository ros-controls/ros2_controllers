^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.10.0 (2025-12-01)
-------------------

5.9.0 (2025-11-10)
------------------
* Controller interface api update to ros2_controller packages (`#1973 <https://github.com/ros-controls/ros2_controllers/issues/1973>`_)
* Fix integer literal for size_t (`#1986 <https://github.com/ros-controls/ros2_controllers/issues/1986>`_)
* Contributors: Anand Vardhan, Christoph Fröhlich

5.8.0 (2025-10-02)
------------------
* Update API for realtime publisher (`#1830 <https://github.com/ros-controls/ros2_controllers/issues/1830>`_)
* Remove deprecated methods from ros2_control (`#1936 <https://github.com/ros-controls/ros2_controllers/issues/1936>`_)
* Contributors: Christoph Fröhlich

5.7.0 (2025-09-12)
------------------

5.6.1 (2025-08-30)
------------------

5.6.0 (2025-08-29)
------------------
* docs(joint_state_broadcaster): clarify /dynamic_joint_states contents (`#1865 <https://github.com/ros-controls/ros2_controllers/issues/1865>`_)
* Contributors: rishitej04

5.5.0 (2025-07-31)
------------------

5.4.0 (2025-07-23)
------------------

5.3.0 (2025-07-14)
------------------

5.2.0 (2025-06-23)
------------------

5.1.0 (2025-06-11)
------------------
* Added frame_id to Joint State Broadcaster (`#1746 <https://github.com/ros-controls/ros2_controllers/issues/1746>`_)
* Fix RST syntax (`#1715 <https://github.com/ros-controls/ros2_controllers/issues/1715>`_)
* Contributors: Christoph Fröhlich, Jakub "Deli" Delicat

5.0.2 (2025-05-26)
------------------
* Fix JSB+GPIO CMakeLists and dependencies (`#1705 <https://github.com/ros-controls/ros2_controllers/issues/1705>`_)
* Contributors: Christoph Fröhlich

5.0.1 (2025-05-24)
------------------
* Use target_link_libraries instead of ament_target_dependencies (`#1697 <https://github.com/ros-controls/ros2_controllers/issues/1697>`_)
* Contributors: Sai Kishor Kothakota

5.0.0 (2025-05-17)
------------------

4.24.0 (2025-04-27)
-------------------
* Call `configure()` of base class instead of node (`#1659 <https://github.com/ros-controls/ros2_controllers/issues/1659>`_)
* Fix joint_state_broadcaster performance issues (`#1640 <https://github.com/ros-controls/ros2_controllers/issues/1640>`_)
* Contributors: Christoph Fröhlich, Jordan Palacios

4.23.0 (2025-04-10)
-------------------
* [JSB] added fixes to mantain the joint names order (`#1572 <https://github.com/ros-controls/ros2_controllers/issues/1572>`_)
* Use global cmake macros and fix gcc-10 build (`#1527 <https://github.com/ros-controls/ros2_controllers/issues/1527>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.22.0 (2025-03-17)
-------------------
* [JSB] cleanup the activation error message (`#1584 <https://github.com/ros-controls/ros2_controllers/issues/1584>`_)
* Contributors: Sai Kishor Kothakota

4.21.0 (2025-03-01)
-------------------
* Cleanup wrong lifecycle transitions in tests and unnecessary checks (`#1534 <https://github.com/ros-controls/ros2_controllers/issues/1534>`_)
* Contributors: Christoph Fröhlich

4.20.0 (2025-01-29)
-------------------
* Update paths of GPL includes (`#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_)
* Contributors: Christoph Fröhlich

4.19.0 (2025-01-13)
-------------------
* Use urdf/model.hpp for rolling (`#1476 <https://github.com/ros-controls/ros2_controllers/issues/1476>`_)
* Use urdf/model.hpp for rolling (`#1473 <https://github.com/ros-controls/ros2_controllers/issues/1473>`_)
* Remove visibility macros (`#1451 <https://github.com/ros-controls/ros2_controllers/issues/1451>`_)
* Contributors: Bence Magyar, verma nakul

4.18.0 (2024-12-19)
-------------------
* [CI] Add clang job and setup concurrency (`#1407 <https://github.com/ros-controls/ros2_controllers/issues/1407>`_)
* Contributors: Christoph Fröhlich

4.17.0 (2024-12-07)
-------------------
* Use the .hpp headers from `realtime_tools` package (`#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_)
* Add few warning flags to error in all ros2_controllers packages and fix tests (`#1370 <https://github.com/ros-controls/ros2_controllers/issues/1370>`_)
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.16.0 (2024-11-08)
-------------------
* [JSB] Fix the behaviour of publishing unavailable state interfaces when they are previously available (`#1331 <https://github.com/ros-controls/ros2_controllers/issues/1331>`_)
* Contributors: Sai Kishor Kothakota

4.15.0 (2024-10-07)
-------------------

4.14.0 (2024-09-11)
-------------------
* [JSB] Move the initialize of urdf::Model from on_activate to on_configure to improve real-time performance (`#1269 <https://github.com/ros-controls/ros2_controllers/issues/1269>`_)
* Contributors: Takashi Sato

4.13.0 (2024-08-22)
-------------------
* [Joint State Broadcaster] Publish the joint_states of joints present in the URDF (`#1233 <https://github.com/ros-controls/ros2_controllers/issues/1233>`_)
* Contributors: Sai Kishor Kothakota

4.12.1 (2024-08-14)
-------------------

4.12.0 (2024-07-23)
-------------------
* Add missing includes (`#1226 <https://github.com/ros-controls/ros2_controllers/issues/1226>`_)
* Change the subscription timeout in the tests to 5ms (`#1219 <https://github.com/ros-controls/ros2_controllers/issues/1219>`_)
* Unused header cleanup (`#1199 <https://github.com/ros-controls/ros2_controllers/issues/1199>`_)
* Fix WaitSet issue in tests  (`#1206 <https://github.com/ros-controls/ros2_controllers/issues/1206>`_)
* Fix parallel gripper controller CI (`#1202 <https://github.com/ros-controls/ros2_controllers/issues/1202>`_)
* Contributors: Christoph Fröhlich, Henry Moore, Sai Kishor Kothakota

4.11.0 (2024-07-09)
-------------------
* added changes corresponding to the logger and clock propagation in ResourceManager (`#1184 <https://github.com/ros-controls/ros2_controllers/issues/1184>`_)
* Contributors: Sai Kishor Kothakota

4.10.0 (2024-07-01)
-------------------

4.9.0 (2024-06-05)
------------------

4.8.0 (2024-05-14)
------------------

4.7.0 (2024-03-22)
------------------
* added conditioning to have rolling tags compilable in older versions (`#1071 <https://github.com/ros-controls/ros2_controllers/issues/1071>`_)
* Contributors: Sai Kishor Kothakota

4.6.0 (2024-02-12)
------------------
* Add test_depend on `hardware_interface_testing` (`#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_)
* Fix tests for using new `get_node_options` API (`#840 <https://github.com/ros-controls/ros2_controllers/issues/840>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.5.0 (2024-01-31)
------------------
* Add tests for `interface_configuration_type` consistently (`#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_)
* Let sphinx add parameter description with nested structures to documentation (`#652 <https://github.com/ros-controls/ros2_controllers/issues/652>`_)
* Contributors: Christoph Fröhlich

4.4.0 (2024-01-11)
------------------

4.3.0 (2024-01-08)
------------------
* Add few warning flags to error (`#961 <https://github.com/ros-controls/ros2_controllers/issues/961>`_)
* Contributors: Sai Kishor Kothakota

4.2.0 (2023-12-12)
------------------

4.1.0 (2023-12-01)
------------------
* Increase test coverage of interface configuration getters (`#856 <https://github.com/ros-controls/ros2_controllers/issues/856>`_)
* joint_state_broadcaster: Add proper subscription to TestCustomInterfaceMappingUpdate (`#859 <https://github.com/ros-controls/ros2_controllers/issues/859>`_)
* Contributors: Christoph Fröhlich

4.0.0 (2023-11-21)
------------------
* fix tests for API break of passing controller manager update rate in init method (`#854 <https://github.com/ros-controls/ros2_controllers/issues/854>`_)
* Adjust tests after passing URDF to controllers (`#817 <https://github.com/ros-controls/ros2_controllers/issues/817>`_)
* Contributors: Bence Magyar, Sai Kishor Kothakota

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
