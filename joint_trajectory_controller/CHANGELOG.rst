^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
