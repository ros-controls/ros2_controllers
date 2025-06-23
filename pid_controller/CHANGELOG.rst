^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pid_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add new members for PID controller parameters (backport `#1585 <https://github.com/ros-controls/ros2_controllers/issues/1585>`_) (`#1769 <https://github.com/ros-controls/ros2_controllers/issues/1769>`_)
* Set enable_feedforward parameter in the respective tests (backport `#1743 <https://github.com/ros-controls/ros2_controllers/issues/1743>`_) (`#1744 <https://github.com/ros-controls/ros2_controllers/issues/1744>`_)
* Contributors: Victor Coutinho Vieira Santos, Sai Kishor Kothakota, Christoph Fröhlich

4.26.0 (2025-06-06)
-------------------
* Use target_link_libraries instead of ament_target_dependencies (backport `#1697 <https://github.com/ros-controls/ros2_controllers/issues/1697>`_) (`#1699 <https://github.com/ros-controls/ros2_controllers/issues/1699>`_)
* Contributors: mergify[bot]

4.25.0 (2025-05-17)
-------------------
* [Pid] Add enable_feedforward parameter (backport `#1553 <https://github.com/ros-controls/ros2_controllers/issues/1553>`_) (`#1689 <https://github.com/ros-controls/ros2_controllers/issues/1689>`_)
* Simplify `on_set_chained_mode` avoiding cpplint warnings (backport `#1564 <https://github.com/ros-controls/ros2_controllers/issues/1564>`_) (`#1688 <https://github.com/ros-controls/ros2_controllers/issues/1688>`_)
* Contributors: mergify[bot], Pascal Auf der Maur, hagyesh Agresar

4.24.0 (2025-04-27)
-------------------

4.23.0 (2025-04-10)
-------------------
* Bump version of pre-commit hooks (`#1618 <https://github.com/ros-controls/ros2_controllers/issues/1618>`_)
* Use global cmake macros and fix gcc-10 build (`#1527 <https://github.com/ros-controls/ros2_controllers/issues/1527>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.22.0 (2025-03-17)
-------------------

4.21.0 (2025-03-01)
-------------------
* [pid_controller] Update tests (`#1538 <https://github.com/ros-controls/ros2_controllers/issues/1538>`_)
* Reset PID controllers on activation and add `save_i_term` parameter (`#1507 <https://github.com/ros-controls/ros2_controllers/issues/1507>`_)
* Update API of PID class (`#1437 <https://github.com/ros-controls/ros2_controllers/issues/1437>`_)
* [pid_controller] Fix logic for feedforward_mode with single reference interface (`#1520 <https://github.com/ros-controls/ros2_controllers/issues/1520>`_)
* Fix the exported interface naming in the chainable controllers (`#1528 <https://github.com/ros-controls/ros2_controllers/issues/1528>`_)
* Contributors: Christoph Fröhlich, Julia Jia, Sai Kishor Kothakota

4.20.0 (2025-01-29)
-------------------
* Improve antiwindup description (`#1502 <https://github.com/ros-controls/ros2_controllers/issues/1502>`_)
* Remove empty callbacks (`#1488 <https://github.com/ros-controls/ros2_controllers/issues/1488>`_)
* Update paths of GPL includes (`#1487 <https://github.com/ros-controls/ros2_controllers/issues/1487>`_)
* Contributors: Christoph Fröhlich, Julia Jia, Victor Coutinho Vieira Santos

4.19.0 (2025-01-13)
-------------------
* Remove visibility macros (`#1451 <https://github.com/ros-controls/ros2_controllers/issues/1451>`_)
* Contributors: Bence Magyar

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
* fixes for windows compilation (`#1330 <https://github.com/ros-controls/ros2_controllers/issues/1330>`_)
* Contributors: Gilmar Correia

4.15.0 (2024-10-07)
-------------------
* Adapt test to new way of exporting reference interfaces (Related to `#1240 <https://github.com/ros-controls/ros2_controllers/issues/1240>`_ in ros2_control) (`#1103 <https://github.com/ros-controls/ros2_controllers/issues/1103>`_)
* Contributors: Manuel Muth

4.14.0 (2024-09-11)
-------------------
* [PID Controller] Export state interfaces for easier chaining with other controllers (`#1214 <https://github.com/ros-controls/ros2_controllers/issues/1214>`_)
* Contributors: Sai Kishor Kothakota

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
* [PID] Add example yaml to docs (`#951 <https://github.com/ros-controls/ros2_controllers/issues/951>`_)
* Contributors: Christoph Fröhlich

4.7.0 (2024-03-22)
------------------
* Fix pid_controller build on ROS 2 Rolling on Ubuntu 24.04 (`#1084 <https://github.com/ros-controls/ros2_controllers/issues/1084>`_)
* Added conditioning to have rolling tags compilable in older versions (`#1071 <https://github.com/ros-controls/ros2_controllers/issues/1071>`_)
* Fix usage of visibility macros (`#1039 <https://github.com/ros-controls/ros2_controllers/issues/1039>`_)
* Contributors: Chris Lalancette, Sai Kishor Kothakota, Silvio Traversaro

4.6.0 (2024-02-12)
------------------
* Add test_depend on `hardware_interface_testing` (`#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_)
* Fix tests for using new `get_node_options` API (`#840 <https://github.com/ros-controls/ros2_controllers/issues/840>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.5.0 (2024-01-31)
------------------
* Add tests for `interface_configuration_type` consistently (`#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_)
* [PID] Remove joint_jog include (`#975 <https://github.com/ros-controls/ros2_controllers/issues/975>`_)
* Contributors: Christoph Fröhlich

4.4.0 (2024-01-11)
------------------

4.3.0 (2024-01-08)
------------------
* Add few warning flags to error (`#961 <https://github.com/ros-controls/ros2_controllers/issues/961>`_)
* Contributors: Sai Kishor Kothakota

4.2.0 (2023-12-12)
------------------
* 🚀 Add PID controller 🎉 (`#434 <https://github.com/ros-controls/ros2_controllers/issues/434>`_)
* Contributors: Dr. Denis

4.1.0 (2023-12-01)
------------------

4.0.0 (2023-11-21)
------------------

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

3.10.1 (2023-06-06)
-------------------

3.10.0 (2023-06-04)
-------------------

3.9.0 (2023-05-28)
------------------

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
