^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gripper_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.28.0 (2024-11-13)
-------------------
* Update maintainers and add url tags (backport `#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_) (`#1365 <https://github.com/ros-controls/ros2_controllers/issues/1365>`_)
* Contributors: mergify[bot]

3.27.0 (2024-11-02)
-------------------

3.26.3 (2024-09-11)
-------------------

3.26.2 (2024-08-22)
-------------------

3.26.1 (2024-08-14)
-------------------

3.26.0 (2024-07-24)
-------------------

3.25.0 (2024-07-09)
-------------------

3.24.0 (2024-05-14)
-------------------

3.23.0 (2024-04-30)
-------------------

3.22.0 (2024-02-12)
-------------------
* Add test_depend on `hardware_interface_testing` (backport `#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_) (`#1020 <https://github.com/ros-controls/ros2_controllers/issues/1020>`_)
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
* Contributors: Christoph Froehlich

3.18.0 (2023-11-21)
-------------------

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
