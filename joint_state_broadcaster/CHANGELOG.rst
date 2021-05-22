^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
