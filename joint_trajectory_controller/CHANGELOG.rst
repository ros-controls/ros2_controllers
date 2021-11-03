^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2021-10-25)
------------------
* Move interface sorting into ControllerInterface (`#259 <https://github.com/ros-controls/ros2_controllers/issues/259>`_)
* Revise for-loop style (`#254 <https://github.com/ros-controls/ros2_controllers/issues/254>`_)
* Contributors: bailaC

1.0.0 (2021-09-29)
------------------
* Remove compile warnings. (`#245 <https://github.com/ros-controls/ros2_controllers/issues/245>`_)
* Add time and period to update function (`#241 <https://github.com/ros-controls/ros2_controllers/issues/241>`_)
* Quickfix 🛠: Correct confusing variable name (`#240 <https://github.com/ros-controls/ros2_controllers/issues/240>`_)
* Unify style of controllers. (`#236 <https://github.com/ros-controls/ros2_controllers/issues/236>`_)
* Change test to work with Foxy and posterior action API (`#237 <https://github.com/ros-controls/ros2_controllers/issues/237>`_)
* ros2_controllers code changes to support ros2_controls issue `#489 <https://github.com/ros-controls/ros2_controllers/issues/489>`_ (`#233 <https://github.com/ros-controls/ros2_controllers/issues/233>`_)
* Removing Boost from controllers. (`#235 <https://github.com/ros-controls/ros2_controllers/issues/235>`_)
* refactor get_current_state to get_state (`#232 <https://github.com/ros-controls/ros2_controllers/issues/232>`_)
* Contributors: Bence Magyar, Denis Štogl, Márk Szitanics, Tyler Weaver, bailaC

0.5.0 (2021-08-30)
------------------
* Add auto declaration of parameters. (`#224 <https://github.com/ros-controls/ros2_controllers/issues/224>`_)
* Bring precommit config up to speed with ros2_control (`#227 <https://github.com/ros-controls/ros2_controllers/issues/227>`_)
* Add initial pre-commit setup. (`#220 <https://github.com/ros-controls/ros2_controllers/issues/220>`_)
* Enable JTC for hardware having offset from state measurements (`#189 <https://github.com/ros-controls/ros2_controllers/issues/189>`_)
  * Avoid "jumps" with states that have tracking error. All test are passing but separatelly. Is there some kind of timeout?
  * Remove allow_integration_flag
  * Add reading from command interfaces when restarting controller
* Reduce docs warnings and correct adding guidelines (`#219 <https://github.com/ros-controls/ros2_controllers/issues/219>`_)
* Contributors: Bence Magyar, Denis Štogl, Lovro Ivanov

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------
* Force torque sensor broadcaster (`#152 <https://github.com/ros-controls/ros2_controllers/issues/152>`_)
  * Stabilize joint_trajectory_controller tests
  * Add  rclcpp::shutdown(); to all standalone test functions
* Fixes for Windows (`#205 <https://github.com/ros-controls/ros2_controllers/issues/205>`_)
  * Export protected joint trajectory controller functions
* Fix deprecation warnings on Rolling, remove rcutils dependency (`#204 <https://github.com/ros-controls/ros2_controllers/issues/204>`_)
* Fix parameter initialisation for galactic (`#199 <https://github.com/ros-controls/ros2_controllers/issues/199>`_)
  * Fix parameter initialisation for galactic
  * Fix forward_command_controller the same way
  * Fix other compiler warnings
  * Missing space
* Fix rolling build (`#200 <https://github.com/ros-controls/ros2_controllers/issues/200>`_)
  * Fix rolling build
  * Stick to printf style
  * Add back :: around interface type
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* Contributors: Akash, Bence Magyar, Denis Štogl, Tim Clephas, Vatan Aksoy Tezer

0.3.1 (2021-05-23)
------------------
* Reset external trajectory message upon activation (`#185 <https://github.com/ros-controls/ros2_controllers/issues/185>`_)
  * Reset external trajectory message to prevent preserving the old goal on systems with hardware offsets
  * Fix has_trajectory_msg() function: two wrongs were making a right so functionally things were fine
* Contributors: Nathan Brooks, Matt Reynolds

0.3.0 (2021-05-21)
------------------
* joint_trajectory_controller publishes state in node namespace (`#187 <https://github.com/ros-controls/ros2_controllers/issues/187>`_)
* [JointTrajectoryController] Enable position, velocity and acceleration interfaces (`#140 <https://github.com/ros-controls/ros2_controllers/issues/140>`_)
  * joint_trajectory_controller should not go into FINALIZED state when fails to configure, remain in UNCONFIGURED
* Contributors: Bence Magyar, Denis Štogl

0.2.1 (2021-05-03)
------------------
* Migrate from deprecated controller_interface::return_type::SUCCESS -> OK (`#167 <https://github.com/ros-controls/ros2_controllers/issues/167>`_)
* [JTC] Add link to TODOs to provide better trackability (`#169 <https://github.com/ros-controls/ros2_controllers/issues/169>`_)
* Fix JTC segfault (`#164 <https://github.com/ros-controls/ros2_controllers/issues/164>`_)
  * Use a copy of the rt_active_goal to avoid segfault
  * Use RealtimeBuffer for thread-safety
* Add basic user docs pages for each package (`#156 <https://github.com/ros-controls/ros2_controllers/issues/156>`_)
* Contributors: Bence Magyar, Matt Reynolds

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

0.1.0 (2020-12-23)
------------------
* Remove lifecycle node controllers (`#124 <https://github.com/ros-controls/ros2_controllers/issues/124>`_)
* Use resource manager on joint trajectory controller (`#112 <https://github.com/ros-controls/ros2_controllers/issues/112>`_)
* Use new joint handles in all controllers (`#90 <https://github.com/ros-controls/ros2_controllers/issues/90>`_)
* More jtc tests (`#75 <https://github.com/ros-controls/ros2_controllers/issues/75>`_)
* remove unused variables (`#86 <https://github.com/ros-controls/ros2_controllers/issues/86>`_)
* Port over interpolation formulae, abort if goals tolerance violated (`#62 <https://github.com/ros-controls/ros2_controllers/issues/62>`_)
* Partial joints (`#68 <https://github.com/ros-controls/ros2_controllers/issues/68>`_)
* Use clamp function from rcppmath (`#79 <https://github.com/ros-controls/ros2_controllers/issues/79>`_)
* Reorder incoming out of order joint_names in trajectory messages (`#53 <https://github.com/ros-controls/ros2_controllers/issues/53>`_)
* Action server for JointTrajectoryController (`#26 <https://github.com/ros-controls/ros2_controllers/issues/26>`_)
* Add state_publish_rate to JointTrajectoryController (`#25 <https://github.com/ros-controls/ros2_controllers/issues/25>`_)
* Contributors: Alejandro Hernández Cordero, Anas Abou Allaban, Bence Magyar, Denis Štogl, Edwin Fan, Jordan Palacios, Karsten Knese, Victor Lopez
