^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package steering_controllers_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.28.0 (2025-07-14)
-------------------

4.27.1 (2025-07-02)
-------------------
* Fix SteeringOdometry calculation error (backport `#1777 <https://github.com/ros-controls/ros2_controllers/issues/1777>`_) (`#1779 <https://github.com/ros-controls/ros2_controllers/issues/1779>`_)
* Contributors: mergify[bot]

4.27.0 (2025-06-23)
-------------------

4.26.0 (2025-06-06)
-------------------
* Fix steering_controllers_library docs (`#1734 <https://github.com/ros-controls/ros2_controllers/issues/1734>`_)
* Use target_link_libraries instead of ament_target_dependencies (backport `#1697 <https://github.com/ros-controls/ros2_controllers/issues/1697>`_) (`#1699 <https://github.com/ros-controls/ros2_controllers/issues/1699>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

4.25.0 (2025-05-17)
-------------------
* Simplify `on_set_chained_mode` avoiding cpplint warnings (backport `#1564 <https://github.com/ros-controls/ros2_controllers/issues/1564>`_) (`#1688 <https://github.com/ros-controls/ros2_controllers/issues/1688>`_)
* Contributors: mergify[bot], Bhagyesh Agresar

4.24.0 (2025-04-27)
-------------------
* Rename ackermann msg to controller state msg type (`#1662 <https://github.com/ros-controls/ros2_controllers/issues/1662>`_)
* Remove front_steering from steering library (`#1166 <https://github.com/ros-controls/ros2_controllers/issues/1166>`_)
* Fix preceeding->preceding typos (`#1655 <https://github.com/ros-controls/ros2_controllers/issues/1655>`_)
* Contributors: Christoph Fröhlich, Enrique Llorente Pastora, Mukunda Bharatheesha

4.23.0 (2025-04-10)
-------------------
* Bump version of pre-commit hooks (`#1618 <https://github.com/ros-controls/ros2_controllers/issues/1618>`_)
* Use global cmake macros and fix gcc-10 build (`#1527 <https://github.com/ros-controls/ros2_controllers/issues/1527>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.22.0 (2025-03-17)
-------------------

4.21.0 (2025-03-01)
-------------------
* Fix the exported interface naming in the chainable controllers (`#1528 <https://github.com/ros-controls/ros2_controllers/issues/1528>`_)
* Fix use of M_PI in steering_controllers_library and tricycle_controller (`#1536 <https://github.com/ros-controls/ros2_controllers/issues/1536>`_)
* Contributors: Sai Kishor Kothakota, Silvio Traversaro

4.20.0 (2025-01-29)
-------------------
* Update paths of GPL includes (`#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_)
* Contributors: Christoph Fröhlich

4.19.0 (2025-01-13)
-------------------
* Update generate_parameter_library dependency in steering_controllers_library (`#1465 <https://github.com/ros-controls/ros2_controllers/issues/1465>`_)
* Fix typos in steering_controllers_lib (`#1464 <https://github.com/ros-controls/ros2_controllers/issues/1464>`_)
* Fix open-loop odometry in case of ref timeout (`#1454 <https://github.com/ros-controls/ros2_controllers/issues/1454>`_)
* Remove visibility macros (`#1451 <https://github.com/ros-controls/ros2_controllers/issues/1451>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, Silvio Traversaro

4.18.0 (2024-12-19)
-------------------
* steering_controllers_library: Add `reduce_wheel_speed_until_steering_reached` parameter (`#1314 <https://github.com/ros-controls/ros2_controllers/issues/1314>`_)
* [CI] Add clang job and setup concurrency (`#1407 <https://github.com/ros-controls/ros2_controllers/issues/1407>`_)
* Contributors: Christoph Fröhlich

4.17.0 (2024-12-07)
-------------------
* Use the .hpp headers from `realtime_tools` package (`#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_)
* Add Mecanum Drive Controller (`#512 <https://github.com/ros-controls/ros2_controllers/issues/512>`_)
* Add few warning flags to error in all ros2_controllers packages and fix tests (`#1370 <https://github.com/ros-controls/ros2_controllers/issues/1370>`_)
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_)
* Contributors: Christoph Fröhlich, Dr. Denis, Sai Kishor Kothakota

4.16.0 (2024-11-08)
-------------------

4.15.0 (2024-10-07)
-------------------
* Adapt test to new way of exporting reference interfaces (Related to `#1240 <https://github.com/ros-controls/ros2_controllers/issues/1240>`_ in ros2_control) (`#1103 <https://github.com/ros-controls/ros2_controllers/issues/1103>`_)
* fix(timeout): do not reset steer wheels to 0. on timeout (`#1289 <https://github.com/ros-controls/ros2_controllers/issues/1289>`_)
* fix(steering-odometry): convert twist to steering angle (`#1288 <https://github.com/ros-controls/ros2_controllers/issues/1288>`_)
* Contributors: Manuel Muth, Rein Appeldoorn

4.14.0 (2024-09-11)
-------------------
* fix(steering-odometry): handle infinite turning radius properly (`#1285 <https://github.com/ros-controls/ros2_controllers/issues/1285>`_)
* Contributors: Rein Appeldoorn

4.13.0 (2024-08-22)
-------------------

4.12.1 (2024-08-14)
-------------------

4.12.0 (2024-07-23)
-------------------
* Add missing includes (`#1226 <https://github.com/ros-controls/ros2_controllers/issues/1226>`_)
* Change the subscription timeout in the tests to 5ms (`#1219 <https://github.com/ros-controls/ros2_controllers/issues/1219>`_)
* Unused header cleanup (`#1199 <https://github.com/ros-controls/ros2_controllers/issues/1199>`_)
* Fix WaitSet issue in tests  (`#1206 <https://github.com/ros-controls/ros2_controllers/issues/1206>`_)
* Contributors: Christoph Fröhlich, Henry Moore, Sai Kishor Kothakota

4.11.0 (2024-07-09)
-------------------
* Fix steering controllers library kinematics (`#1150 <https://github.com/ros-controls/ros2_controllers/issues/1150>`_)
* Contributors: Christoph Fröhlich

4.10.0 (2024-07-01)
-------------------
* [STEERING] Add missing `tan` call for ackermann (`#1117 <https://github.com/ros-controls/ros2_controllers/issues/1117>`_)
* [Steering controllers library] Reference interfaces are body twist (`#1168 <https://github.com/ros-controls/ros2_controllers/issues/1168>`_)
* Fix steering controllers library code documentation and naming (`#1149 <https://github.com/ros-controls/ros2_controllers/issues/1149>`_)
* Remove unstamped twist subscribers + parameters (`#1151 <https://github.com/ros-controls/ros2_controllers/issues/1151>`_)
* Contributors: Christoph Fröhlich, Enrique Llorente Pastora, Quique Llorente

4.9.0 (2024-06-05)
------------------
* Add mobile robot kinematics 101 and improve steering library docs (`#954 <https://github.com/ros-controls/ros2_controllers/issues/954>`_)
* Fix correct usage of angular velocity in update_odometry() function (`#1118 <https://github.com/ros-controls/ros2_controllers/issues/1118>`_)
* Contributors: Christoph Fröhlich, Ferry Schoenmakers

4.8.0 (2024-05-14)
------------------
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
* Fix tests for using new `get_node_options` API (`#840 <https://github.com/ros-controls/ros2_controllers/issues/840>`_)
* Contributors: Sai Kishor Kothakota, Silvio Traversaro

4.5.0 (2024-01-31)
------------------
* Add tests for `interface_configuration_type` consistently (`#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_)
* Contributors: Christoph Fröhlich

4.4.0 (2024-01-11)
------------------

4.3.0 (2024-01-08)
------------------
* Add few warning flags to error (`#961 <https://github.com/ros-controls/ros2_controllers/issues/961>`_)
* Fix ackermann steering odometry (`#921 <https://github.com/ros-controls/ros2_controllers/issues/921>`_)
* Changing default int values to double in steering controller's yaml file (`#927 <https://github.com/ros-controls/ros2_controllers/issues/927>`_)
* Contributors: Franz Rammerstorfer, Reza Kermani, Sai Kishor Kothakota

4.2.0 (2023-12-12)
------------------

4.1.0 (2023-12-01)
------------------

4.0.0 (2023-11-21)
------------------
* fix tests for API break of passing controller manager update rate in init method (`#854 <https://github.com/ros-controls/ros2_controllers/issues/854>`_)
* Adjust tests after passing URDF to controllers (`#817 <https://github.com/ros-controls/ros2_controllers/issues/817>`_)
* Contributors: Bence Magyar, Sai Kishor Kothakota

3.17.0 (2023-10-31)
-------------------
* Steering controllers library: fix open loop mode (`#793 <https://github.com/ros-controls/ros2_controllers/issues/793>`_)
  * set last*velocity variables for open loop odometry
  * Make function arguments const
  * Update function in header file too
* Improve docs (`#785 <https://github.com/ros-controls/ros2_controllers/issues/785>`_)
* Contributors: Christoph Fröhlich

3.16.0 (2023-09-20)
-------------------

3.15.0 (2023-09-11)
-------------------

3.14.0 (2023-08-16)
-------------------

3.13.0 (2023-08-04)
-------------------
* Update ci-ros-lint.yml and copyright format (`#720 <https://github.com/ros-controls/ros2_controllers/issues/720>`_)
* Contributors: Christoph Fröhlich

3.12.0 (2023-07-18)
-------------------

3.11.0 (2023-06-24)
-------------------
* Added -Wconversion flag and fix warnings (`#667 <https://github.com/ros-controls/ros2_controllers/issues/667>`_)
* Let sphinx add parameter description to documentation (`#651 <https://github.com/ros-controls/ros2_controllers/issues/651>`_)
* Contributors: Christoph Fröhlich, gwalck

3.10.1 (2023-06-06)
-------------------
* Second round of dependencies fix (`#655 <https://github.com/ros-controls/ros2_controllers/issues/655>`_)
* Contributors: Bence Magyar

3.10.0 (2023-06-04)
-------------------
* Remove unnecessary include (`#645 <https://github.com/ros-controls/ros2_controllers/issues/645>`_)
* enable ReflowComments to also use ColumnLimit on comments (`#625 <https://github.com/ros-controls/ros2_controllers/issues/625>`_)
* Contributors: Bence Magyar, Sai Kishor Kothakota

3.9.0 (2023-05-28)
------------------
* Fix sphinx for steering odometry library/controllers (`#626 <https://github.com/ros-controls/ros2_controllers/issues/626>`_)
* Steering odometry library and controllers (`#484 <https://github.com/ros-controls/ros2_controllers/issues/484>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, Tomislav Petković

3.8.0 (2023-05-14)
------------------

3.7.0 (2023-05-02)
------------------

3.6.0 (2023-04-29)
------------------

3.5.0 (2023-04-14)
------------------

3.4.0 (2023-04-02)
------------------

3.3.0 (2023-03-07)
------------------

3.2.0 (2023-02-10)
------------------

3.1.0 (2023-01-26)
------------------

3.0.0 (2023-01-19)
------------------

2.15.0 (2022-12-06)
-------------------

2.14.0 (2022-11-18)
-------------------

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------

2.11.0 (2022-08-04)
-------------------

2.10.0 (2022-08-01)
-------------------

2.9.0 (2022-07-14)
------------------

2.8.0 (2022-07-09)
------------------

2.7.0 (2022-07-03)
------------------

2.6.0 (2022-06-18)
------------------

2.5.0 (2022-05-13)
------------------

2.4.0 (2022-04-29)
------------------

2.3.0 (2022-04-21)
------------------

2.2.0 (2022-03-25)
------------------

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

0.5.0 (2021-08-30)
------------------

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------

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
