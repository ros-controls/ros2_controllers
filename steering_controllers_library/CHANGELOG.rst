^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package steering_controllers_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.26.3 (2024-09-11)
-------------------

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
* Fix steering controllers library kinematics (`#1150 <https://github.com/ros-controls/ros2_controllers/issues/1150>`_) (`#1195 <https://github.com/ros-controls/ros2_controllers/issues/1195>`_)
* [Steering controllers library] Reference interfaces are body twist (`#1168 <https://github.com/ros-controls/ros2_controllers/issues/1168>`_) (`#1174 <https://github.com/ros-controls/ros2_controllers/issues/1174>`_)
* [STEERING] Add missing `tan` call for ackermann (`#1117 <https://github.com/ros-controls/ros2_controllers/issues/1117>`_) (`#1177 <https://github.com/ros-controls/ros2_controllers/issues/1177>`_)
* Fix steering controllers library code documentation and naming (`#1149 <https://github.com/ros-controls/ros2_controllers/issues/1149>`_) (`#1165 <https://github.com/ros-controls/ros2_controllers/issues/1165>`_)
* Add mobile robot kinematics 101 and improve steering library docs (`#954 <https://github.com/ros-controls/ros2_controllers/issues/954>`_) (`#1161 <https://github.com/ros-controls/ros2_controllers/issues/1161>`_)
* Fix deprecation warning (`#1155 <https://github.com/ros-controls/ros2_controllers/issues/1155>`_)
* Fix correct usage of angular velocity in update_odometry() function (`#1118 <https://github.com/ros-controls/ros2_controllers/issues/1118>`_) (`#1154 <https://github.com/ros-controls/ros2_controllers/issues/1154>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

3.24.0 (2024-05-14)
-------------------
* Deprecate non-stamped twist for tricycle_controller and steering_controllers (`#1093 <https://github.com/ros-controls/ros2_controllers/issues/1093>`_) (`#1124 <https://github.com/ros-controls/ros2_controllers/issues/1124>`_)
* Contributors: mergify[bot]

3.23.0 (2024-04-30)
-------------------

3.22.0 (2024-02-12)
-------------------
* Fix usage of M_PI on Windows (`#1036 <https://github.com/ros-controls/ros2_controllers/issues/1036>`_) (`#1038 <https://github.com/ros-controls/ros2_controllers/issues/1038>`_)
* Add tests for `interface_configuration_type` consistently (backport `#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_) (`#1007 <https://github.com/ros-controls/ros2_controllers/issues/1007>`_)
* Contributors: mergify[bot]

3.21.0 (2024-01-20)
-------------------

3.20.2 (2024-01-11)
-------------------

3.20.1 (2024-01-08)
-------------------
* Fix ackermann steering odometry (`#921 <https://github.com/ros-controls/ros2_controllers/issues/921>`_) (`#956 <https://github.com/ros-controls/ros2_controllers/issues/956>`_)
* Contributors: mergify[bot]

3.20.0 (2024-01-03)
-------------------
* Changing default int values to double in steering controller's yaml file (`#927 <https://github.com/ros-controls/ros2_controllers/issues/927>`_) (`#929 <https://github.com/ros-controls/ros2_controllers/issues/929>`_)
* Contributors: mergify[bot]

3.19.2 (2023-12-12)
-------------------

3.19.1 (2023-12-05)
-------------------

3.19.0 (2023-12-01)
-------------------

3.18.0 (2023-11-21)
-------------------

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
