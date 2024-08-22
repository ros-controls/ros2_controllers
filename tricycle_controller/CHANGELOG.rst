^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tricycle_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.26.1 (2024-08-14)
-------------------

3.26.0 (2024-07-24)
-------------------
* Fix WaitSet issue in tests  (backport `#1206 <https://github.com/ros-controls/ros2_controllers/issues/1206>`_) (`#1212 <https://github.com/ros-controls/ros2_controllers/issues/1212>`_)
* Contributors: mergify[bot]

3.25.0 (2024-07-09)
-------------------
* Add mobile robot kinematics 101 and improve steering library docs (`#954 <https://github.com/ros-controls/ros2_controllers/issues/954>`_) (`#1161 <https://github.com/ros-controls/ros2_controllers/issues/1161>`_)
* Bump version of pre-commit hooks (`#1157 <https://github.com/ros-controls/ros2_controllers/issues/1157>`_) (`#1159 <https://github.com/ros-controls/ros2_controllers/issues/1159>`_)
* Contributors: mergify[bot]

3.24.0 (2024-05-14)
-------------------
* Deprecate non-stamped twist for tricycle_controller and steering_controllers (`#1093 <https://github.com/ros-controls/ros2_controllers/issues/1093>`_) (`#1124 <https://github.com/ros-controls/ros2_controllers/issues/1124>`_)
* Add parameter check for geometric values (`#1120 <https://github.com/ros-controls/ros2_controllers/issues/1120>`_) (`#1126 <https://github.com/ros-controls/ros2_controllers/issues/1126>`_)
* Contributors: mergify[bot]

3.23.0 (2024-04-30)
-------------------
* [tricycle_controller] Use generate_parameter_library (backport `#957 <https://github.com/ros-controls/ros2_controllers/issues/957>`_) (`#991 <https://github.com/ros-controls/ros2_controllers/issues/991>`_)
* Contributors: mergify[bot]

3.22.0 (2024-02-12)
-------------------
* Fix usage of M_PI on Windows (`#1036 <https://github.com/ros-controls/ros2_controllers/issues/1036>`_) (`#1038 <https://github.com/ros-controls/ros2_controllers/issues/1038>`_)
* Add test_depend on `hardware_interface_testing` (backport `#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_) (`#1020 <https://github.com/ros-controls/ros2_controllers/issues/1020>`_)
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
