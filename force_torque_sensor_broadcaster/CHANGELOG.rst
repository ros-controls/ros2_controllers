^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package force_torque_sensor_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add tests for `interface_configuration_type` consistently (`#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_) (`#1011 <https://github.com/ros-controls/ros2_controllers/issues/1011>`_)
* Revert "[ForceTorqueSensorBroadcaster] Create ParamListener and get parameters on configure (`#698 <https://github.com/ros-controls/ros2_controllers/issues/698>`_)" (`#988 <https://github.com/ros-controls/ros2_controllers/issues/988>`_) (`#1003 <https://github.com/ros-controls/ros2_controllers/issues/1003>`_)
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
* [ForceTorqueSensorBroadcaster] Create ParamListener and get parameters on configure (backport `#698 <https://github.com/ros-controls/ros2_controllers/issues/698>`_) (`#750 <https://github.com/ros-controls/ros2_controllers/issues/750>`_)
  * [ForceTorqueSensorBroadcaster] Create ParamListener and get parameters on configure (`#698 <https://github.com/ros-controls/ros2_controllers/issues/698>`_)
  * Create ParamListener and get parameters on configure
  * Declare parameters for test_force_torque_sensor_broadcaster
  Since the parameters are not declared on init anymore, they cannot be
  set without declaring them before
  ---------
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  (cherry picked from commit 32aaef7552638826aba0b3f3a72b1c1453739afa)
  * Fix "parameter is already declared" error
  ---------
  Co-authored-by: Noel Jiménez García <noel.jimenez@pal-robotics.com>
  Co-authored-by: Christoph Froehlich <christoph.froehlich@ait.ac.at>
* Contributors: mergify[bot]

2.28.0 (2023-11-30)
-------------------

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
* Broadcaster parameters (`#650 <https://github.com/ros-controls/ros2_controllers/issues/650>`_) (`#678 <https://github.com/ros-controls/ros2_controllers/issues/678>`_)
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_) (`#677 <https://github.com/ros-controls/ros2_controllers/issues/677>`_)
* Contributors: Christoph Fröhlich, Bence Magyar

2.22.0 (2023-06-14)
-------------------
* Docs: Use branch name substitution for all links (backport `#618 <https://github.com/ros-controls/ros2_controllers/issues/618>`_) (`#633 <https://github.com/ros-controls/ros2_controllers/issues/633>`_)
* Contributors: Christoph Fröhlich

2.21.0 (2023-05-28)
-------------------
* Fix github links on control.ros.org (`#604 <https://github.com/ros-controls/ros2_controllers/issues/604>`_) (`#617 <https://github.com/ros-controls/ros2_controllers/issues/617>`_)
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_) (`#605 <https://github.com/ros-controls/ros2_controllers/issues/605>`_)
* Contributors: Felix Exner (fexner), Christoph Fröhlich

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

2.17.1 (2023-02-20)
-------------------

2.17.0 (2023-02-13)
-------------------

2.16.1 (2023-01-31)
-------------------

2.16.0 (2023-01-19)
-------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_) (`#493 <https://github.com/ros-controls/ros2_controllers/issues/493>`_)
* Contributors: Bence Magyar

2.15.0 (2022-12-06)
-------------------

2.14.0 (2022-11-18)
-------------------
* Fix parameter library export (`#448 <https://github.com/ros-controls/ros2_controllers/issues/448>`_)
* Contributors: Tyler Weaver

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------
* Generate params for ForceTorqueSensorBroadcaster (`#395 <https://github.com/ros-controls/ros2_controllers/issues/395>`_)
* Contributors: Tyler Weaver

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
* Disable failing workflows (`#363 <https://github.com/ros-controls/ros2_controllers/issues/363>`_)
* CMakeLists cleanup (`#362 <https://github.com/ros-controls/ros2_controllers/issues/362>`_)
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Andy Zelenak, Jafar Abdi

2.5.0 (2022-05-13)
------------------
* fix: :bug: make force_torque_sensor_broadcaster wait for realtime_publisher (`#327 <https://github.com/ros-controls/ros2_controllers/issues/327>`_)
* Contributors: Jaron Lundwall, Denis Štogl

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
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* Contributors: Bence Magyar, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Contributors: Bence Magyar, Denis Štogl, livanov93

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Fix dependency (`#208 <https://github.com/ros-controls/ros2_controllers/issues/208>`_)
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Stabilize joint_trajectory_controller tests
  * Add  rclcpp::shutdown(); to all standalone test functions
* Contributors: Bence Magyar, Denis Štogl, Nisala Kalupahana, Subhas Das

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
