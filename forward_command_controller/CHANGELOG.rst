^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package forward_command_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
