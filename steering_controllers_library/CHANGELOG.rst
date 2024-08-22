^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package steering_controllers_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
