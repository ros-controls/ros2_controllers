^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gripper_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.43.0 (2025-03-17)
-------------------

2.42.1 (2025-02-24)
-------------------

2.42.0 (2025-02-17)
-------------------
* Bump version of pre-commit hooks (backport `#1514 <https://github.com/ros-controls/ros2_controllers/issues/1514>`_) (`#1515 <https://github.com/ros-controls/ros2_controllers/issues/1515>`_)
* Update paths of GPL includes (backport `#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_) (`#1493 <https://github.com/ros-controls/ros2_controllers/issues/1493>`_)
* Contributors: Christoph Fröhlich

2.41.0 (2025-01-13)
-------------------

2.40.0 (2025-01-01)
-------------------
* Use the .hpp headers from `realtime_tools` package (backport `#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_) (`#1427 <https://github.com/ros-controls/ros2_controllers/issues/1427>`_)
* Contributors: mergify[bot]

2.39.0 (2024-12-03)
-------------------
* Add explicit cast to period.count() (`#1404 <https://github.com/ros-controls/ros2_controllers/issues/1404>`_) (`#1405 <https://github.com/ros-controls/ros2_controllers/issues/1405>`_)
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_) (`#1364 <https://github.com/ros-controls/ros2_controllers/issues/1364>`_)
* Contributors: mergify[bot]

2.38.0 (2024-11-09)
-------------------
* Added -Wconversion flag and fix warnings (`#667 <https://github.com/ros-controls/ros2_controllers/issues/667>`_) (`#1321 <https://github.com/ros-controls/ros2_controllers/issues/1321>`_)
* Contributors: mergify[bot]

2.37.3 (2024-09-11)
-------------------

2.37.2 (2024-08-22)
-------------------

2.37.1 (2024-08-14)
-------------------

2.37.0 (2024-07-24)
-------------------

2.36.0 (2024-07-09)
-------------------

2.35.0 (2024-05-22)
-------------------

2.34.0 (2024-04-01)
-------------------
* Let sphinx add parameter description with nested structures to documentation (backport `#652 <https://github.com/ros-controls/ros2_controllers/issues/652>`_) (`#1005 <https://github.com/ros-controls/ros2_controllers/issues/1005>`_)
* Contributors: mergify[bot]

2.33.0 (2024-02-12)
-------------------
* Add test_depend on `hardware_interface_testing` (backport `#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_) (`#1019 <https://github.com/ros-controls/ros2_controllers/issues/1019>`_)
* Contributors: mergify[bot]

2.32.0 (2024-01-20)
-------------------
* Increase test coverage of interface configuration getters (`#856 <https://github.com/ros-controls/ros2_controllers/issues/856>`_) (`#865 <https://github.com/ros-controls/ros2_controllers/issues/865>`_)
* Contributors: mergify[bot]

2.31.0 (2024-01-11)
-------------------

2.30.0 (2023-12-20)
-------------------

2.29.0 (2023-12-05)
-------------------
* Add test for effort gripper controller (`#769 <https://github.com/ros-controls/ros2_controllers/issues/769>`_) (`#867 <https://github.com/ros-controls/ros2_controllers/issues/867>`_)
* Contributors: mergify[bot]

2.28.0 (2023-11-30)
-------------------
* Fixed implementation so that effort_controllers/GripperActionController works. (`#756 <https://github.com/ros-controls/ros2_controllers/issues/756>`_) (`#868 <https://github.com/ros-controls/ros2_controllers/issues/868>`_)
* Contributors: mergify[bot]

2.27.0 (2023-11-14)
-------------------

2.26.0 (2023-10-03)
-------------------

2.25.0 (2023-09-15)
-------------------

2.24.0 (2023-08-07)
-------------------

2.23.0 (2023-06-23)
-------------------
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_) (`#677 <https://github.com/ros-controls/ros2_controllers/issues/677>`_)
* Contributors: Bence Magyar

2.22.0 (2023-06-14)
-------------------
* Let sphinx add parameter description to documentation (backport `#651 <https://github.com/ros-controls/ros2_controllers/issues/651>`_) (`#663 <https://github.com/ros-controls/ros2_controllers/issues/663>`_)
* [JTC] Fix missing parameter deprecation warnings (`#630 <https://github.com/ros-controls/ros2_controllers/issues/630>`_)
* Contributors: Noel Jiménez García, Christoph Fröhlich

2.21.0 (2023-05-28)
-------------------
* Fix compilation warnings (`#621 <https://github.com/ros-controls/ros2_controllers/issues/621>`_) (`#623 <https://github.com/ros-controls/ros2_controllers/issues/623>`_)
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_) (`#605 <https://github.com/ros-controls/ros2_controllers/issues/605>`_)
* Contributors: Felix Exner (fexner), Christoph Fröhlich, Mathias Lüdtke, Noel Jiménez García

2.20.0 (2023-05-14)
-------------------

2.19.0 (2023-05-02)
-------------------

2.18.0 (2023-04-29)
-------------------

2.17.3 (2023-04-14)
-------------------

2.17.2 (2023-03-07)
-------------------
* Fix Segfault in GripperActionController (`#527 <https://github.com/ros-controls/ros2_controllers/issues/527>`_) (`#530 <https://github.com/ros-controls/ros2_controllers/issues/530>`_)
* Contributors: Erik Holum, Bence Magyar

2.17.1 (2023-02-20)
-------------------

2.17.0 (2023-02-13)
-------------------

2.16.1 (2023-01-31)
-------------------
* Changing to_chrono to use nanoseconds (`#507 <https://github.com/ros-controls/ros2_controllers/issues/507>`_) (`#509 <https://github.com/ros-controls/ros2_controllers/issues/509>`_)
* Contributors: Dan Wahl

2.16.0 (2023-01-19)
-------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_) (`#493 <https://github.com/ros-controls/ros2_controllers/issues/493>`_)
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
