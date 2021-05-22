^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2021-05-21)
------------------

0.2.1 (2021-05-03)
------------------
* Rename joint_state_controller -> joint_state_broadcaster (`#160 <https://github.com/ros-controls/ros2_controllers/issues/160>`_)
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
* avoid warnings (`#137 <https://github.com/ros-controls/ros2_controllers/issues/137>`_)
* Contributors: Karsten Knese

0.1.0 (2020-12-23)
------------------
* Print exception msg (`#130 <https://github.com/ros-controls/ros2_controllers/issues/130>`_)
* Remove lifecycle node controllers (`#124 <https://github.com/ros-controls/ros2_controllers/issues/124>`_)
* joint state controller with resource manager (`#109 <https://github.com/ros-controls/ros2_controllers/issues/109>`_)
* Use new joint handles in all controllers (`#90 <https://github.com/ros-controls/ros2_controllers/issues/90>`_)
* Clock() defaults to RCL_SYSTEM_TIME without considering parameter use_sim_time. (`#83 <https://github.com/ros-controls/ros2_controllers/issues/83>`_)
* Joint states are now published using the DynamicJointState message (`#65 <https://github.com/ros-controls/ros2_controllers/issues/65>`_)
* Add on_activate to JointStateController (`#43 <https://github.com/ros-controls/ros2_controllers/issues/43>`_)
* Contributors: Alejandro Hernández Cordero, Bence Magyar, Denis Štogl, Hang, Jordan Palacios, Karsten Knese, Victor Lopez, Yutaka Kondo
