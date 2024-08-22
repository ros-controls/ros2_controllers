^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package forward_command_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

3.24.0 (2024-05-14)
-------------------

3.23.0 (2024-04-30)
-------------------

3.22.0 (2024-02-12)
-------------------
* Add test_depend on `hardware_interface_testing` (backport `#1018 <https://github.com/ros-controls/ros2_controllers/issues/1018>`_) (`#1020 <https://github.com/ros-controls/ros2_controllers/issues/1020>`_)
* Add tests for `interface_configuration_type` consistently (backport `#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_) (`#1007 <https://github.com/ros-controls/ros2_controllers/issues/1007>`_)
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
* Rearrange controllers overview page (`#846 <https://github.com/ros-controls/ros2_controllers/issues/846>`_) (`#848 <https://github.com/ros-controls/ros2_controllers/issues/848>`_)
* Contributors: mergify[bot]

3.17.0 (2023-10-31)
-------------------

3.16.0 (2023-09-20)
-------------------
* [Doc] Add specific documentation on the available fw cmd controllers (`#765 <https://github.com/ros-controls/ros2_controllers/issues/765>`_)
* Contributors: Christoph Fröhlich

3.15.0 (2023-09-11)
-------------------

3.14.0 (2023-08-16)
-------------------
* Use tabs (`#743 <https://github.com/ros-controls/ros2_controllers/issues/743>`_)
* Contributors: Christoph Fröhlich

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
* Fix docs format (`#589 <https://github.com/ros-controls/ros2_controllers/issues/589>`_)
* Contributors: Bence Magyar, Christoph Fröhlich

3.5.0 (2023-04-14)
------------------

3.4.0 (2023-04-02)
------------------
* Fix broken links (`#554 <https://github.com/ros-controls/ros2_controllers/issues/554>`_)
* Update docs (`#552 <https://github.com/ros-controls/ros2_controllers/issues/552>`_)
* Contributors: Christoph Fröhlich

3.3.0 (2023-03-07)
------------------
* Add comments about auto-generated header files (`#539 <https://github.com/ros-controls/ros2_controllers/issues/539>`_)
* Contributors: AndyZe

3.2.0 (2023-02-10)
------------------
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_)
* Contributors: Tyler Weaver, Chris Thrasher

3.1.0 (2023-01-26)
------------------

3.0.0 (2023-01-19)
------------------
* Add backward_ros to all controllers (`#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_)
* Contributors: Bence Magyar

2.15.0 (2022-12-06)
-------------------

2.14.0 (2022-11-18)
-------------------
* Generate params for ForwardCommandController (`#396 <https://github.com/ros-controls/ros2_controllers/issues/396>`_)
* Contributors: Tyler Weaver

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
* CMakeLists cleanup (`#362 <https://github.com/ros-controls/ros2_controllers/issues/362>`_)
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Andy Zelenak, Jafar Abdi

2.5.0 (2022-05-13)
------------------

2.4.0 (2022-04-29)
------------------
* Multi-interface Forward Controller (`#154 <https://github.com/ros-controls/ros2_controllers/issues/154>`_)
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
* Adding reset() for forward_command_controller (`#283 <https://github.com/ros-controls/ros2_controllers/issues/283>`_)
* Contributors: bailaC

1.2.0 (2021-12-29)
------------------
* Forward command controller test update (`#273 <https://github.com/ros-controls/ros2_controllers/issues/273>`_)
  * removed unnecessary lines and updated comments
  * fixed pre-commit issues
  * removed extra part of test
* Contributors: Jack Center

1.1.0 (2021-10-25)
------------------
* Move interface sorting into ControllerInterface (`#259 <https://github.com/ros-controls/ros2_controllers/issues/259>`_)
* Revise for-loop style (`#254 <https://github.com/ros-controls/ros2_controllers/issues/254>`_)
* Contributors: bailaC

1.0.0 (2021-09-29)
------------------
* Reset and test of command buffer for forwarding controllers. (`#246 <https://github.com/ros-controls/ros2_controllers/issues/246>`_)
* Remove compile warnings. (`#245 <https://github.com/ros-controls/ros2_controllers/issues/245>`_)
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* Contributors: Bence Magyar, Denis Štogl, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Reduce docs warnings and correct adding guidelines (`#219 <https://github.com/ros-controls/ros2_controllers/issues/219>`_)
* Contributors: Bence Magyar, Denis Štogl, Lovro Ivanov

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Add  rclcpp::shutdown(); to all standalone test functions
* Fix parameter initialisation for galactic (`#199 <https://github.com/ros-controls/ros2_controllers/issues/199>`_)
* Contributors: Denis Štogl, Tim Clephas

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------
* [JointTrajectoryController] Enable position, velocity and acceleration interfaces (`#140 <https://github.com/ros-controls/ros2_controllers/issues/140>`_)
  * joint_trajectory_controller should not go into FINALIZED state when fails to configure, remain in UNCONFIGURED
* Contributors: Denis Štogl, Bence Magyar

0.2.1 (2021-05-03)
------------------
* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Add basic user docs pages for each package (`#156 <https://github.com/ros-controls/ros2_controllers/issues/156>`_)
* Contributors: Bence Magyar

0.2.0 (2021-02-06)
------------------
* Use ros2 contol test assets (`#138 <https://github.com/ros-controls/ros2_controllers/issues/138>`_)
  * Add description to test trajecotry_controller
  * Use ros2_control_test_assets package
  * Delete obsolete components plugin export
* Contributors: Denis Štogl

0.1.2 (2021-01-07)
------------------

0.1.1 (2021-01-06)
------------------
* Restore forward command derivatives (`#133 <https://github.com/ros-controls/ros2_controllers/issues/133>`_)
* Contributors: Bence Magyar

0.1.0 (2020-12-23)
------------------
* ForwardCommandController declares parameters (`#131 <https://github.com/ros-controls/ros2_controllers/issues/131>`_)
* Remove lifecycle node controllers (`#124 <https://github.com/ros-controls/ros2_controllers/issues/124>`_)
* joint state controller with resource manager (`#109 <https://github.com/ros-controls/ros2_controllers/issues/109>`_)
* forward_command_controller (`#87 <https://github.com/ros-controls/ros2_controllers/issues/87>`_)
* Contributors: Bence Magyar, Jordan Palacios, Karsten Knese, Victor Lopez
