^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gripper_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

4.24.0 (2025-04-27)
-------------------

4.23.0 (2025-04-10)
-------------------
* Use global cmake macros and fix gcc-10 build (`#1527 <https://github.com/ros-controls/ros2_controllers/issues/1527>`_)
* Replace RCLCPP\_*_STREAM macros with RCLCPP\_* (`#1600 <https://github.com/ros-controls/ros2_controllers/issues/1600>`_)
* Contributors: Christoph Fröhlich, Vedant Randive

4.22.0 (2025-03-17)
-------------------

4.21.0 (2025-03-01)
-------------------
* Update API of PID class (`#1437 <https://github.com/ros-controls/ros2_controllers/issues/1437>`_)
* Cleanup wrong lifecycle transitions in tests and unnecessary checks (`#1534 <https://github.com/ros-controls/ros2_controllers/issues/1534>`_)
* Bump version of pre-commit hooks (`#1514 <https://github.com/ros-controls/ros2_controllers/issues/1514>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.20.0 (2025-01-29)
-------------------
* Update paths of GPL includes (`#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_)
* Contributors: Christoph Fröhlich

4.19.0 (2025-01-13)
-------------------
* Remove custom logic to skip configuration of gripper_controllers on Windows or macOS (`#1471 <https://github.com/ros-controls/ros2_controllers/issues/1471>`_)
* Remove visibility macros (`#1451 <https://github.com/ros-controls/ros2_controllers/issues/1451>`_)
* Contributors: Bence Magyar, Silvio Traversaro

4.18.0 (2024-12-19)
-------------------

4.17.0 (2024-12-07)
-------------------
* Use the .hpp headers from `realtime_tools` package (`#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_)
* Add explicit cast to period.count() (`#1404 <https://github.com/ros-controls/ros2_controllers/issues/1404>`_)
* Add few warning flags to error in all ros2_controllers packages and fix tests (`#1370 <https://github.com/ros-controls/ros2_controllers/issues/1370>`_)
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.16.0 (2024-11-08)
-------------------

4.15.0 (2024-10-07)
-------------------

4.14.0 (2024-09-11)
-------------------

4.13.0 (2024-08-22)
-------------------

4.12.1 (2024-08-14)
-------------------

4.12.0 (2024-07-23)
-------------------
* Unused header cleanup (`#1199 <https://github.com/ros-controls/ros2_controllers/issues/1199>`_)
* Fix parallel gripper controller CI (`#1202 <https://github.com/ros-controls/ros2_controllers/issues/1202>`_)
* Add parallel_gripper_controller, configure gripper speed and effort with hardware interface (`#1002 <https://github.com/ros-controls/ros2_controllers/issues/1002>`_)
* Contributors: Henry Moore, Paul Gesel, Sai Kishor Kothakota

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
* Fix usage of visibility macros (`#1039 <https://github.com/ros-controls/ros2_controllers/issues/1039>`_)
* Contributors: Silvio Traversaro

4.6.0 (2024-02-12)
------------------
* Add test_depend on `hardware_interface_testing` (`#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_)
* Fix tests for using new `get_node_options` API (`#840 <https://github.com/ros-controls/ros2_controllers/issues/840>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.5.0 (2024-01-31)
------------------
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
* Add test for effort gripper controller (`#769 <https://github.com/ros-controls/ros2_controllers/issues/769>`_)
* Fixed implementation so that effort_controllers/GripperActionController works. (`#756 <https://github.com/ros-controls/ros2_controllers/issues/756>`_)
* Contributors: chama1176

3.14.0 (2023-08-16)
-------------------

3.13.0 (2023-08-04)
-------------------

3.12.0 (2023-07-18)
-------------------

3.11.0 (2023-06-24)
-------------------
* Added -Wconversion flag and fix warnings (`#667 <https://github.com/ros-controls/ros2_controllers/issues/667>`_)
* Let sphinx add parameter description to documentation (`#651 <https://github.com/ros-controls/ros2_controllers/issues/651>`_)
* Contributors: Christoph Fröhlich, gwalck

3.10.1 (2023-06-06)
-------------------

3.10.0 (2023-06-04)
-------------------

3.9.0 (2023-05-28)
------------------
* Fix compilation warnings (`#621 <https://github.com/ros-controls/ros2_controllers/issues/621>`_)
* Contributors: Noel Jiménez García, Mathias Lüdtke

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
* [Parameters] Use `gt_eq` instead of deprecated `lower_bounds` in validators (`#561 <https://github.com/ros-controls/ros2_controllers/issues/561>`_)
* Contributors: Dr. Denis

3.4.0 (2023-04-02)
------------------

3.3.0 (2023-03-07)
------------------
* Add comments about auto-generated header files (`#539 <https://github.com/ros-controls/ros2_controllers/issues/539>`_)
* Fix Segfault in GripperActionController (`#527 <https://github.com/ros-controls/ros2_controllers/issues/527>`_)
* Contributors: AndyZe, Erik Holum

3.2.0 (2023-02-10)
------------------
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_)
* Contributors: Tyler Weaver, Chris Thrasher

3.1.0 (2023-01-26)
------------------
* Changing to_chrono to use nanoseconds & Reset gripper action goal timer to match JTC impl (`#507 <https://github.com/ros-controls/ros2_controllers/issues/507>`_)
* Contributors: Dan Wahl

3.0.0 (2023-01-19)
------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_)
* Contributors: Bence Magyar

2.15.0 (2022-12-06)
-------------------
* Add basic gripper controller tests (`#459 <https://github.com/ros-controls/ros2_controllers/issues/459>`_)
* Contributors: Bence Magyar

2.14.0 (2022-11-18)
-------------------
* Use optional from C++17 (`#460 <https://github.com/ros-controls/ros2_controllers/issues/460>`_)
* Generate parameters for Gripper Action (`#398 <https://github.com/ros-controls/ros2_controllers/issues/398>`_)
* Contributors: Bence Magyar, Tyler Weaver

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------
* Add an initialization of the gripper action command for safe startup. (`#425 <https://github.com/ros-controls/ros2_controllers/issues/425>`_)
* Fix formatting CI job (`#418 <https://github.com/ros-controls/ros2_controllers/issues/418>`_)
* Contributors: Shota Aoki, Tyler Weaver

2.11.0 (2022-08-04)
-------------------

2.10.0 (2022-08-01)
-------------------
* Formatting changes from pre-commit (`#400 <https://github.com/ros-controls/ros2_controllers/issues/400>`_)
* Parameter loading fixup in diff_drive and gripper controllers (`#385 <https://github.com/ros-controls/ros2_controllers/issues/385>`_)
* Contributors: Andy Zelenak, Tyler Weaver

2.9.0 (2022-07-14)
------------------
* Allow gripper stalling when moving to goal (`#355 <https://github.com/ros-controls/ros2_controllers/issues/355>`_)
* Contributors: Marq Rasmussen

2.8.0 (2022-07-09)
------------------

2.7.0 (2022-07-03)
------------------
* Update controllers with new get_name hardware interfaces (`#369 <https://github.com/ros-controls/ros2_controllers/issues/369>`_)
* Contributors: Lucas Schulze

2.6.0 (2022-06-18)
------------------
* CMakeLists cleanup (`#362 <https://github.com/ros-controls/ros2_controllers/issues/362>`_)
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Andy Zelenak, Jafar Abdi

2.5.0 (2022-05-13)
------------------

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

2.0.1 (2022-02-01)
------------------

2.0.0 (2022-01-28)
------------------

1.3.0 (2022-01-11)
------------------

1.2.0 (2021-12-29)
------------------

1.1.0 (2021-10-25)
------------------

1.0.0 (2021-09-29)
------------------
* Remove compile warnings. (`#245 <https://github.com/ros-controls/ros2_controllers/issues/245>`_)
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* Unify style of controllers. (`#236 <https://github.com/ros-controls/ros2_controllers/issues/236>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* Contributors: Bence Magyar, Denis Štogl, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* Contributors: Bence Magyar, Lovro Ivanov

0.4.1 (2021-07-08)
------------------
* Fix test dependencies (`#213 <https://github.com/ros-controls/ros2_controllers/issues/213>`_)
* Contributors: Bence Magyar

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Add  rclcpp::shutdown(); to all standalone test functions
* Fixes for Windows (`#205 <https://github.com/ros-controls/ros2_controllers/issues/205>`_)
  * Disable gripper on Windows too
* disable gripper on OSX (`#192 <https://github.com/ros-controls/ros2_controllers/issues/192>`_)
* Port gripper action controller to ROS2 (`#162 <https://github.com/ros-controls/ros2_controllers/issues/162>`_)
* Contributors: Bence Magyar, Denis Štogl, Jafar Abdi

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------

0.2.1 (2021-05-03)
------------------

0.2.0 (2021-02-06)
------------------

0.1.2 (2021-01-07)
------------------

0.1.1 (2021-01-06)
------------------

0.1.0 (2020-12-23)
------------------
