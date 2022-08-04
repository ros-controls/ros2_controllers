^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.0 (2022-08-04)
-------------------
* Use explicit type in joint_state_broadcaster test (`#403 <https://github.com/ros-controls/ros2_controllers/issues/403>`_)
  This use of `auto` is causing a static assert on RHEL. Explicitly
  specifying the type seems to resolve the failure and allow the test to
  be compiled.
* Contributors: Scott K Logan

2.10.0 (2022-08-01)
-------------------

2.9.0 (2022-07-14)
------------------

2.8.0 (2022-07-09)
------------------

2.7.0 (2022-07-03)
------------------
* Update controllers with new get_name hardware interfaces (`#369 <https://github.com/ros-controls/ros2_controllers/issues/369>`_)
* Contributors: Lucas Schulze

2.6.0 (2022-06-18)
------------------
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Contributors: Jafar Abdi

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
* Use lifecycle node as base for controllers (`#244 <https://github.com/ros-controls/ros2_controllers/issues/244>`_)
* Contributors: Denis Štogl, Vatan Aksoy Tezer, Bence Magyar

2.1.0 (2022-02-23)
------------------
* joint_state_broadcaster to use realtime tools (`#276 <https://github.com/ros-controls/ros2_controllers/issues/276>`_)
* Contributors: Bence Magyar

2.0.1 (2022-02-01)
------------------

2.0.0 (2022-01-28)
------------------

1.3.0 (2022-01-11)
------------------

1.2.0 (2021-12-29)
------------------
* [Joint State Broadcaster] Add mapping of custom states to standard values in "/joint_state" message (`#217 <https://github.com/ros-controls/ros2_controllers/issues/217>`_)
* [Joint State Broadcaster] Add option to support only specific interfaces on specific joints (`#216 <https://github.com/ros-controls/ros2_controllers/issues/216>`_)
* Contributors: Denis Štogl, Bence Magyar

1.1.0 (2021-10-25)
------------------
* Revise for-loop style (`#254 <https://github.com/ros-controls/ros2_controllers/issues/254>`_)
* Contributors: bailaC

1.0.0 (2021-09-29)
------------------
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* Unify style of controllers. (`#236 <https://github.com/ros-controls/ros2_controllers/issues/236>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* Contributors: Bence Magyar, Denis Štogl, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* [Joint State Broadcaster] Add option to publish joint states to local topics (`#218 <https://github.com/ros-controls/ros2_controllers/issues/218>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Reduce docs warnings and correct adding guidelines (`#219 <https://github.com/ros-controls/ros2_controllers/issues/219>`_)
* Contributors: Bence Magyar, Denis Štogl, Lovro Ivanov

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Add  rclcpp::shutdown(); to all standalone test functions
* Contributors: Denis Štogl

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------
* Remove unused variable (`#181 <https://github.com/ros-controls/ros2_controllers/issues/181>`_)
* Add extra joints parameter at joint state broadcaster (`#179 <https://github.com/ros-controls/ros2_controllers/issues/179>`_)
* Contributors: Cesc Folch Aldehuelo, Karsten Knese

0.2.1 (2021-05-03)
------------------
* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Rename joint_state_controller -> joint_state_broadcaster (`#160 <https://github.com/ros-controls/ros2_controllers/issues/160>`_)
  * Rename joint_state_controller -> _broadcaster
  * Update accompanying files (Ament, CMake, etc)
  * Update C++ from _controller to _broadcaster
  * Apply cpplint
  * Create stub controller to redirect to _broadcaster
  * Add test for loading old joint_state_controller
  * Add missing dependency on hardware_interface
  * Add link to documentation
  * Add joint_state_broadcaster to metapackage
  * Apply suggestions from code review
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update joint_state_broadcaster/joint_state_plugin.xml
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* Contributors: Bence Magyar, Matt Reynolds

* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* Rename joint_state_controller -> joint_state_broadcaster (`#160 <https://github.com/ros-controls/ros2_controllers/issues/160>`_)
  * Rename joint_state_controller -> _broadcaster
  * Update accompanying files (Ament, CMake, etc)
  * Update C++ from _controller to _broadcaster
  * Apply cpplint
  * Create stub controller to redirect to _broadcaster
  * Add test for loading old joint_state_controller
  * Add missing dependency on hardware_interface
  * Add link to documentation
  * Add joint_state_broadcaster to metapackage
  * Apply suggestions from code review
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update joint_state_broadcaster/joint_state_plugin.xml
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
* Contributors: Bence Magyar, Matt Reynolds

0.2.0 (2021-02-06)
------------------

0.1.2 (2021-01-07)
------------------

0.1.1 (2021-01-06)
------------------

0.1.0 (2020-12-23)
------------------
