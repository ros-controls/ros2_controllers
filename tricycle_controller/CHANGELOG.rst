^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tricycle_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.36.0 (2025-12-31)
-------------------
* Controller interface api update to ros2_controller packages (backport `#1973 <https://github.com/ros-controls/ros2_controllers/issues/1973>`_) (`#2068 <https://github.com/ros-controls/ros2_controllers/issues/2068>`_)
* Contributors: mergify[bot]

4.35.0 (2025-12-01)
-------------------

4.34.0 (2025-11-10)
-------------------

4.33.1 (2025-10-17)
-------------------
* Don't use `msg\_` field of realtime publisher (backport `#1947 <https://github.com/ros-controls/ros2_controllers/issues/1947>`_) (`#1948 <https://github.com/ros-controls/ros2_controllers/issues/1948>`_)
* Contributors: mergify[bot]

4.33.0 (2025-10-03)
-------------------
* Update API for realtime publisher (backport `#1830 <https://github.com/ros-controls/ros2_controllers/issues/1830>`_) (`#1944 <https://github.com/ros-controls/ros2_controllers/issues/1944>`_)
* Use new handles API in ros2_controllers to fix deprecation warnings (backport `#1566 <https://github.com/ros-controls/ros2_controllers/issues/1566>`_) (`#1934 <https://github.com/ros-controls/ros2_controllers/issues/1934>`_)
* Fix: Remove deprecated `rclcpp::spin_some(node)` (backport `#1928 <https://github.com/ros-controls/ros2_controllers/issues/1928>`_) (`#1932 <https://github.com/ros-controls/ros2_controllers/issues/1932>`_)
* Contributors: mergify[bot]

4.32.0 (2025-09-12)
-------------------

4.31.0 (2025-08-27)
-------------------

4.30.1 (2025-08-03)
-------------------

4.30.0 (2025-07-31)
-------------------

4.29.0 (2025-07-23)
-------------------

4.28.0 (2025-07-14)
-------------------

4.27.1 (2025-07-02)
-------------------

4.27.0 (2025-06-23)
-------------------

4.26.0 (2025-06-06)
-------------------
* Use target_link_libraries instead of ament_target_dependencies (backport `#1697 <https://github.com/ros-controls/ros2_controllers/issues/1697>`_) (`#1699 <https://github.com/ros-controls/ros2_controllers/issues/1699>`_)
* Contributors: mergify[bot]

4.25.0 (2025-05-17)
-------------------
* Deprecating tf2 C Headers (`#1325 <https://github.com/ros-controls/ros2_controllers/issues/1325>`_)
* Contributors: Lucas Wendland

4.24.0 (2025-04-27)
-------------------
* Call `configure()` of base class instead of node (`#1659 <https://github.com/ros-controls/ros2_controllers/issues/1659>`_)
* Contributors: Christoph Fröhlich

4.23.0 (2025-04-10)
-------------------
* Bump version of pre-commit hooks (`#1618 <https://github.com/ros-controls/ros2_controllers/issues/1618>`_)
* Use global cmake macros and fix gcc-10 build (`#1527 <https://github.com/ros-controls/ros2_controllers/issues/1527>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.22.0 (2025-03-17)
-------------------

4.21.0 (2025-03-01)
-------------------
* Cleanup wrong lifecycle transitions in tests and unnecessary checks (`#1534 <https://github.com/ros-controls/ros2_controllers/issues/1534>`_)
* Fix use of M_PI in steering_controllers_library and tricycle_controller (`#1536 <https://github.com/ros-controls/ros2_controllers/issues/1536>`_)
* Contributors: Christoph Fröhlich, Silvio Traversaro

4.20.0 (2025-01-29)
-------------------
* Update paths of GPL includes (`#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_)
* Contributors: Christoph Fröhlich

4.19.0 (2025-01-13)
-------------------
* Remove empty on_shutdown() callbacks (`#1477 <https://github.com/ros-controls/ros2_controllers/issues/1477>`_)
* Remove visibility macros (`#1451 <https://github.com/ros-controls/ros2_controllers/issues/1451>`_)
* Contributors: Bence Magyar, Julia Jia

4.18.0 (2024-12-19)
-------------------
* [CI] Add clang job and setup concurrency (`#1407 <https://github.com/ros-controls/ros2_controllers/issues/1407>`_)
* Contributors: Christoph Fröhlich

4.17.0 (2024-12-07)
-------------------
* Use the .hpp headers from `realtime_tools` package (`#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_)
* Fix RealtimeBox API changes (`#1385 <https://github.com/ros-controls/ros2_controllers/issues/1385>`_)
* Add few warning flags to error in all ros2_controllers packages and fix tests (`#1370 <https://github.com/ros-controls/ros2_controllers/issues/1370>`_)
* TractionLimiter: Fix wrong input checks (`#1341 <https://github.com/ros-controls/ros2_controllers/issues/1341>`_)
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.16.0 (2024-11-08)
-------------------

4.15.0 (2024-10-07)
-------------------

4.14.0 (2024-09-11)
-------------------
* rename get/set_state to get/set_lifecylce_state (`#1250 <https://github.com/ros-controls/ros2_controllers/issues/1250>`_)
* Contributors: Manuel Muth

4.13.0 (2024-08-22)
-------------------
* Fixes tests to work with use_global_arguments NodeOptions parameter  (`#1256 <https://github.com/ros-controls/ros2_controllers/issues/1256>`_)
* Contributors: Sai Kishor Kothakota

4.12.1 (2024-08-14)
-------------------

4.12.0 (2024-07-23)
-------------------
* Add missing includes (`#1226 <https://github.com/ros-controls/ros2_controllers/issues/1226>`_)
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
* Remove unstamped twist subscribers + parameters (`#1151 <https://github.com/ros-controls/ros2_controllers/issues/1151>`_)
* Contributors: Christoph Fröhlich

4.9.0 (2024-06-05)
------------------
* Add mobile robot kinematics 101 and improve steering library docs (`#954 <https://github.com/ros-controls/ros2_controllers/issues/954>`_)
* Bump version of pre-commit hooks (`#1157 <https://github.com/ros-controls/ros2_controllers/issues/1157>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.8.0 (2024-05-14)
------------------
* Add parameter check for geometric values (`#1120 <https://github.com/ros-controls/ros2_controllers/issues/1120>`_)
* Deprecate non-stamped twist for tricycle_controller and steering_controllers (`#1093 <https://github.com/ros-controls/ros2_controllers/issues/1093>`_)
* add missing compiler definitions of RCPPUTILS_VERSION (`#1089 <https://github.com/ros-controls/ros2_controllers/issues/1089>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.7.0 (2024-03-22)
------------------
* added conditioning to have rolling tags compilable in older versions (`#1071 <https://github.com/ros-controls/ros2_controllers/issues/1071>`_)
* Fix usage of visibility macros (`#1039 <https://github.com/ros-controls/ros2_controllers/issues/1039>`_)
* Contributors: Sai Kishor Kothakota, Silvio Traversaro

4.6.0 (2024-02-12)
------------------
* Fix usage of M_PI on Windows (`#1036 <https://github.com/ros-controls/ros2_controllers/issues/1036>`_)
* Add test_depend on `hardware_interface_testing` (`#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_)
* Fix tests for using new `get_node_options` API (`#840 <https://github.com/ros-controls/ros2_controllers/issues/840>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota, Silvio Traversaro

4.5.0 (2024-01-31)
------------------
* [tricycle_controller] Use generate_parameter_library (`#957 <https://github.com/ros-controls/ros2_controllers/issues/957>`_)
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
* enable ReflowComments to also use ColumnLimit on comments (`#625 <https://github.com/ros-controls/ros2_controllers/issues/625>`_)
* Contributors: Sai Kishor Kothakota

3.9.0 (2023-05-28)
------------------
* Use branch name substitution for all links (`#618 <https://github.com/ros-controls/ros2_controllers/issues/618>`_)
* Fix github links on control.ros.org (`#604 <https://github.com/ros-controls/ros2_controllers/issues/604>`_)
* Contributors: Christoph Fröhlich

3.8.0 (2023-05-14)
------------------

3.7.0 (2023-05-02)
------------------

3.6.0 (2023-04-29)
------------------
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_)
* Contributors: Bence Magyar

3.5.0 (2023-04-14)
------------------

3.4.0 (2023-04-02)
------------------

3.3.0 (2023-03-07)
------------------
* Use std::clamp instead of rcppmath::clamp (`#540 <https://github.com/ros-controls/ros2_controllers/issues/540>`_)
* Remove publish_rate argument (`#529 <https://github.com/ros-controls/ros2_controllers/issues/529>`_)
* Contributors: Christoph Fröhlich, Tony Najjar

3.2.0 (2023-02-10)
------------------
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_)
* Contributors: Tyler Weaver, Chris Thrasher

3.1.0 (2023-01-26)
------------------

3.0.0 (2023-01-19)
------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_)
* Fix deprecation warnings when compiling (`#478 <https://github.com/ros-controls/ros2_controllers/issues/478>`_)
* Contributors: Bence Magyar, Denis Štogl

2.15.0 (2022-12-06)
-------------------
* [TricycleController] Removed “publish period” functionality ⏱ #abi-break #behavior-break (`#468 <https://github.com/ros-controls/ros2_controllers/issues/468>`_)
* Contributors: Robotgir, Denis Štogl

2.14.0 (2022-11-18)
-------------------
* Include <string> to fix compilation error on macOS (`#467 <https://github.com/ros-controls/ros2_controllers/issues/467>`_)
* Contributors: light-tech

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------
* Fix formatting CI job (`#418 <https://github.com/ros-controls/ros2_controllers/issues/418>`_)
* Fix formatting because pre-commit was not running on CI for some time. (`#409 <https://github.com/ros-controls/ros2_controllers/issues/409>`_)
* Contributors: Denis Štogl, Tyler Weaver

2.11.0 (2022-08-04)
-------------------
* Tricycle controller (`#345 <https://github.com/ros-controls/ros2_controllers/issues/345>`_)
* Contributors: Bence Magyar, Tony Najjar
