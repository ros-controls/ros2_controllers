^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tricycle_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
