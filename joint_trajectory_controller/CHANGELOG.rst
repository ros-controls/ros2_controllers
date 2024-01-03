^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.30.0 (2023-12-20)
-------------------
* Fix floating point comparison in JTC (backport `#879 <https://github.com/ros-controls/ros2_controllers/issues/879>`_)
* [JTC] Continue with last trajectory-point on success (backport `#842 <https://github.com/ros-controls/ros2_controllers/issues/842>`_)
* [JTC] Remove start_with_holding option (backport `#839 <https://github.com/ros-controls/ros2_controllers/issues/839>`_)
* [JTC] Activate checks for parameter validation backport (`#857 <https://github.com/ros-controls/ros2_controllers/issues/857>`_)
* [JTC] Improve update methods for tests (backport `#858 <https://github.com/ros-controls/ros2_controllers/issues/858>`_)
* [JTC] Fix dynamic reconfigure of tolerances (backport `#849 <https://github.com/ros-controls/ros2_controllers/issues/849>`_)
* [JTC] Remove unused home pose (backport `#845 <https://github.com/ros-controls/ros2_controllers/issues/845>`_)
* [JTC] Activate update of dynamic parameters (backport `#761 <https://github.com/ros-controls/ros2_controllers/issues/761>`_)
* [JTC] Fix tests when state offset is used (backport `#797 <https://github.com/ros-controls/ros2_controllers/issues/797>`_)
* Rename wraparound class variables (backport `#834 <https://github.com/ros-controls/ros2_controllers/issues/834>`_)
* Update requirements of state interfaces (backport `#798 <https://github.com/ros-controls/ros2_controllers/issues/798>`_)
* [JTC] Fix typos, implicit cast, const member functions (backport `#748 <https://github.com/ros-controls/ros2_controllers/issues/748>`_)
* Cleanup comments and unnecessary checks (backport `#803 <https://github.com/ros-controls/ros2_controllers/issues/803>`_)
* [JTC] Add tests for acceleration command interface (backport `#752 <https://github.com/ros-controls/ros2_controllers/issues/752>`_)
* [Docs] Improve interface description of JTC (backport `#770 <https://github.com/ros-controls/ros2_controllers/issues/770>`_)
* [JTC] Add time-out for trajectory interfaces (backport `#609 <https://github.com/ros-controls/ros2_controllers/issues/609>`_)
* [JTC] Fix hold position mode with goal_time>0 (backport `#758 <https://github.com/ros-controls/ros2_controllers/issues/758>`_)
* [JTC] Add note on goal_time=0 in docs (backport `#773 <https://github.com/ros-controls/ros2_controllers/issues/773>`_)
* [JTC] Make most parameters read-only (backport `#771 <https://github.com/ros-controls/ros2_controllers/issues/771>`_)
* [JTC] Tolerance tests + Hold on time violation (backport `#613 <https://github.com/ros-controls/ros2_controllers/issues/613>`_)
* [JTC] Explicitly set hold position (backport `#558 <https://github.com/ros-controls/ros2_controllers/issues/558>`_)
* [Doc] Fix links (backport `#715 <https://github.com/ros-controls/ros2_controllers/issues/715>`_)
* Contributors: Christoph Fröhlich, Dr Denis Stogl, Bence Magyar, Abishalini Sivaraman

2.29.0 (2023-12-05)
-------------------

2.28.0 (2023-11-30)
-------------------

2.27.0 (2023-11-14)
-------------------

2.26.0 (2023-10-03)
-------------------

2.25.0 (2023-09-15)
-------------------
* [JTC] Rename parameter: normalize_error to angle_wraparound (`#772 <https://github.com/ros-controls/ros2_controllers/issues/772>`_) (`#776 <https://github.com/ros-controls/ros2_controllers/issues/776>`_)
* Remove wrong description (`#742 <https://github.com/ros-controls/ros2_controllers/issues/742>`_) (`#747 <https://github.com/ros-controls/ros2_controllers/issues/747>`_)
* [JTC] Update trajectory documentation (`#714 <https://github.com/ros-controls/ros2_controllers/issues/714>`_) (`#741 <https://github.com/ros-controls/ros2_controllers/issues/741>`_)
* Contributors: Christoph Fröhlich

2.24.0 (2023-08-07)
-------------------
* [JTC] Disable use of closed-loop PID adapter if controller is used in open-loop mode. (`#551 <https://github.com/ros-controls/ros2_controllers/issues/551>`_) (`#740 <https://github.com/ros-controls/ros2_controllers/issues/740>`_)
* [JTC] Reject messages with effort fields (`#699 <https://github.com/ros-controls/ros2_controllers/issues/699>`_) (`#719 <https://github.com/ros-controls/ros2_controllers/issues/719>`_) (`#738 <https://github.com/ros-controls/ros2_controllers/issues/738>`_)
* [JTC] Re-enabling test, bugfixing and hardening. Adding a parameter to define when trajectories with non-zero velocity at the end are used. (backport `#705 <https://github.com/ros-controls/ros2_controllers/issues/705>`_) (`#706 <https://github.com/ros-controls/ros2_controllers/issues/706>`_)
* Small improvement in remapping (`#393 <https://github.com/ros-controls/ros2_controllers/issues/393>`_) (`#724 <https://github.com/ros-controls/ros2_controllers/issues/724>`_)
* Contributors: Christoph Fröhlich, Dr. Denis, Bence Magyar

2.23.0 (2023-06-23)
-------------------
* Renovate load controller tests (`#569 <https://github.com/ros-controls/ros2_controllers/issues/569>`_) (`#677 <https://github.com/ros-controls/ros2_controllers/issues/677>`_)
* jtc: fix minor typo in traj validation error msg (`#674 <https://github.com/ros-controls/ros2_controllers/issues/674>`_) (`#676 <https://github.com/ros-controls/ros2_controllers/issues/676>`_)
* Contributors: G.A. vd. Hoorn, Bence Magyar

2.22.0 (2023-06-14)
-------------------
* Docs: Use branch name substitution for all links (backport `#618 <https://github.com/ros-controls/ros2_controllers/issues/618>`_) (`#633 <https://github.com/ros-controls/ros2_controllers/issues/633>`_)
* [JTC] Import docs from wiki.ros.org (backport `#566 <https://github.com/ros-controls/ros2_controllers/issues/566>`_) (`#634 <https://github.com/ros-controls/ros2_controllers/issues/634>`_)
* [Formatting] enable ReflowComments to also use ColumnLimit on comments   (`#628 <https://github.com/ros-controls/ros2_controllers/issues/628>`_)
* Contributors: Sai Kishor Kothakota, Christoph Fröhlich

2.21.0 (2023-05-28)
-------------------
* Deprecations in generate_parameter_library. (`#616 <https://github.com/ros-controls/ros2_controllers/issues/616>`_)
* Remove compile warnings. (`#519 <https://github.com/ros-controls/ros2_controllers/issues/519>`_) (`#620 <https://github.com/ros-controls/ros2_controllers/issues/620>`_)
* ported the joint_trajectory_controller query_state service to ROS2 (backport `#481 <https://github.com/ros-controls/ros2_controllers/issues/481>`_) (`#614 <https://github.com/ros-controls/ros2_controllers/issues/614>`_)
* Fix github links on control.ros.org (`#604 <https://github.com/ros-controls/ros2_controllers/issues/604>`_) (`#617 <https://github.com/ros-controls/ros2_controllers/issues/617>`_)
* Fix overriding of install (`#510 <https://github.com/ros-controls/ros2_controllers/issues/510>`_) (`#605 <https://github.com/ros-controls/ros2_controllers/issues/605>`_)
* Contributors: Dr. Denis, Felix Exner (fexner), Christoph Fröhlich

2.20.0 (2023-05-14)
-------------------

2.19.0 (2023-05-02)
-------------------
* Fix JTC from immediately returning success (`#565 <https://github.com/ros-controls/ros2_controllers/issues/565>`_) (`#592 <https://github.com/ros-controls/ros2_controllers/issues/592>`_)
* Implement new ~/controller_state message (`#578 <https://github.com/ros-controls/ros2_controllers/issues/578>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

2.18.0 (2023-04-29)
-------------------
* Fix docs format (`#591 <https://github.com/ros-controls/ros2_controllers/issues/591>`_)
* [JTC] Configurable joint positon error normalization behavior (`#491 <https://github.com/ros-controls/ros2_controllers/issues/491>`_) (`#579 <https://github.com/ros-controls/ros2_controllers/issues/579>`_)
* Contributors: Christoph Fröhlich, Bence Magyar

2.17.3 (2023-04-14)
-------------------
* [JTC] Add pid gain structure to documentation (`#485 <https://github.com/ros-controls/ros2_controllers/issues/485>`_) (`#543 <https://github.com/ros-controls/ros2_controllers/issues/543>`_)
* Fix markup in userdoc.rst (`#480 <https://github.com/ros-controls/ros2_controllers/issues/480>`_) (`#542 <https://github.com/ros-controls/ros2_controllers/issues/542>`_)
* Contributors: Christoph Fröhlich

2.17.2 (2023-03-07)
-------------------

2.17.1 (2023-02-20)
-------------------

2.17.0 (2023-02-13)
-------------------
* fix interpolation logic (`#516 <https://github.com/ros-controls/ros2_controllers/issues/516>`_) (`#523 <https://github.com/ros-controls/ros2_controllers/issues/523>`_)
* fix JTC segfault (`#518 <https://github.com/ros-controls/ros2_controllers/issues/518>`_) (`#524 <https://github.com/ros-controls/ros2_controllers/issues/524>`_)
* Fix JTC segfault on unload (`#515 <https://github.com/ros-controls/ros2_controllers/issues/515>`_) (`#525 <https://github.com/ros-controls/ros2_controllers/issues/525>`_)
* Contributors: Solomon Wiznitzer, Márk Szitanics, Michael Wiznitzer

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
* Generate Parameter Library for Joint Trajectory Controller (`#384 <https://github.com/ros-controls/ros2_controllers/issues/384>`_)
* Fix rates in JTC userdoc.rst (`#433 <https://github.com/ros-controls/ros2_controllers/issues/433>`_)
* Fix for high CPU usage by JTC in gzserver (`#428 <https://github.com/ros-controls/ros2_controllers/issues/428>`_)
  * Change type cast wall timer period from second to nanoseconds.
  create_wall_timer() expects delay in nanoseconds (duration object) however the type cast to seconds will result in 0 (if duration is less than 1s) and thus causing timer to be fired non stop resulting in very high CPU usage.
  * Reset smartpointer so that create_wall_timer() call can destroy previous trajectory timer.
  node->create_wall_timer() first removes timers associated with expired smartpointers before servicing current request.  The JTC timer pointer gets overwrite only after the create_wall_timer() returns and thus not able to remove previous trajectory timer resulting in upto two timers running for JTC during trajectory execution.  Althougth the previous timer does nothing but still get fired.
* Contributors: Arshad Mehmood, Borong Yuan, Tyler Weaver, Andy Zelenak, Bence Magyar, Denis Štogl

2.12.0 (2022-09-01)
-------------------
* Use a "steady clock" when measuring time differences (`#427 <https://github.com/ros-controls/ros2_controllers/issues/427>`_)
* [JTC] Add additional parameter to enable configuration of interfaces for following controllers in a chain. (`#380 <https://github.com/ros-controls/ros2_controllers/issues/380>`_)
* test: :white_check_mark: fix and add back joint_trajectory_controller state_topic_consistency (`#415 <https://github.com/ros-controls/ros2_controllers/issues/415>`_)
* Reinstate JTC tests (`#391 <https://github.com/ros-controls/ros2_controllers/issues/391>`_)
* [JTC] Hold position if tolerance is violated even during non-active goal (`#368 <https://github.com/ros-controls/ros2_controllers/issues/368>`_)
* Small fixes for JTC. (`#390 <https://github.com/ros-controls/ros2_controllers/issues/390>`_)
  variables in JTC to not clutter other PR with them.
  fixes of updating parameters on renewed configuration of JTC that were missed
* Contributors: Andy Zelenak, Bence Magyar, Denis Štogl, Jaron Lundwall, Michael Wiznitzer

2.11.0 (2022-08-04)
-------------------

2.10.0 (2022-08-01)
-------------------
* Make JTC callbacks methods with clear names (`#397 <https://github.com/ros-controls/ros2_controllers/issues/397>`_) #abi-breaking
* Use system time in all tests to avoid error with different time sources. (`#334 <https://github.com/ros-controls/ros2_controllers/issues/334>`_)
* Contributors: Bence Magyar, Denis Štogl

2.9.0 (2022-07-14)
------------------
* Add option to skip interpolation in the joint trajectory controller (`#374 <https://github.com/ros-controls/ros2_controllers/issues/374>`_)
  * Introduce `InterpolationMethods` structure
  * Use parameters to define interpolation use in JTC
* Contributors: Andy Zelenak

2.8.0 (2022-07-09)
------------------
* Preallocate JTC variables to avoid resizing in realtime loops (`#340 <https://github.com/ros-controls/ros2_controllers/issues/340>`_)
* Contributors: Andy Zelenak

2.7.0 (2022-07-03)
------------------
* Properly retrieve parameters in the Joint Trajectory Controller (`#365 <https://github.com/ros-controls/ros2_controllers/issues/365>`_)
* Rename the "abort" variable in the joint traj controller (`#367 <https://github.com/ros-controls/ros2_controllers/issues/367>`_)
* account for edge case in JTC (`#350 <https://github.com/ros-controls/ros2_controllers/issues/350>`_)
* Contributors: Andy Zelenak, Michael Wiznitzer

2.6.0 (2022-06-18)
------------------
* Disable failing workflows (`#363 <https://github.com/ros-controls/ros2_controllers/issues/363>`_)
* Fixed lof message in joint_trayectory_controller (`#366 <https://github.com/ros-controls/ros2_controllers/issues/366>`_)
* CMakeLists cleanup (`#362 <https://github.com/ros-controls/ros2_controllers/issues/362>`_)
* Fix exception about parameter already been declared & Change default c++ version to 17 (`#360 <https://github.com/ros-controls/ros2_controllers/issues/360>`_)
  * Default C++ version to 17
  * Replace explicit use of declare_paremeter with auto_declare
* Member variable renaming in the Joint Traj Controller (`#361 <https://github.com/ros-controls/ros2_controllers/issues/361>`_)
* Contributors: Alejandro Hernández Cordero, Andy Zelenak, Jafar Abdi

2.5.0 (2022-05-13)
------------------
* check for nans in command interface (`#346 <https://github.com/ros-controls/ros2_controllers/issues/346>`_)
* Contributors: Michael Wiznitzer

2.4.0 (2022-04-29)
------------------
* Fix a gtest deprecation warning (`#341 <https://github.com/ros-controls/ros2_controllers/issues/341>`_)
* Delete unused variable in joint_traj_controller (`#339 <https://github.com/ros-controls/ros2_controllers/issues/339>`_)
* updated to use node getter functions (`#329 <https://github.com/ros-controls/ros2_controllers/issues/329>`_)
* Fix JTC state tolerance and goal_time tolerance check bug (`#316 <https://github.com/ros-controls/ros2_controllers/issues/316>`_)
  * fix state tolerance check bug
  * hold position when canceling or aborting. update state tolerance test
  * add goal tolerance fail test
  * better state tolerance test
  * use predefined constants
  * fix goal_time logic and tests
  * add comments
* Contributors: Andy Zelenak, Jack Center, Michael Wiznitzer, Bence Magyar, Denis Štogl

2.3.0 (2022-04-21)
------------------
* [JTC] Allow integration of states in goal trajectories (`#190 <https://github.com/ros-controls/ros2_controllers/issues/190>`_)
  * Added position and velocity deduction to trajectory.
  * Added support for deduction of states from their derivatives.
* Use CallbackReturn from controller_interface namespace (`#333 <https://github.com/ros-controls/ros2_controllers/issues/333>`_)
* [JTC] Implement effort-only command interface (`#225 <https://github.com/ros-controls/ros2_controllers/issues/225>`_)
  * Fix trajectory tolerance parameters
  * Implement effort command interface for JTC
  * Use auto_declare for pid params
  * Set effort to 0 on deactivate
* [JTC] Variable renaming for clearer API (`#323 <https://github.com/ros-controls/ros2_controllers/issues/323>`_)
* Remove unused include to fix JTC test (`#319 <https://github.com/ros-controls/ros2_controllers/issues/319>`_)
* Contributors: Akash, Andy Zelenak, Bence Magyar, Denis Štogl, Jafar Abdi, Victor Lopez

2.2.0 (2022-03-25)
------------------
* Use lifecycle node as base for controllers (`#244 <https://github.com/ros-controls/ros2_controllers/issues/244>`_)
* JointTrajectoryController: added missing control_toolbox dependencies (`#315 <https://github.com/ros-controls/ros2_controllers/issues/315>`_)
* Use time argument on update function instead of node time (`#296 <https://github.com/ros-controls/ros2_controllers/issues/296>`_)
* Export dependency (`#310 <https://github.com/ros-controls/ros2_controllers/issues/310>`_)
* Contributors: DasRoteSkelett, Erick G. Islas-Osuna, Jafar Abdi, Denis Štogl, Vatan Aksoy Tezer, Bence Magyar

2.1.0 (2022-02-23)
------------------
* INSTANTIATE_TEST_CASE_P -> INSTANTIATE_TEST_SUITE_P (`#293 <https://github.com/ros-controls/ros2_controllers/issues/293>`_)
* Contributors: Bence Magyar

2.0.1 (2022-02-01)
------------------
* Fix missing control_toolbox dependency (`#291 <https://github.com/ros-controls/ros2_controllers/issues/291>`_)
* Contributors: Denis Štogl

2.0.0 (2022-01-28)
------------------
* [JointTrajectoryController] Add velocity-only command option for JTC with closed loop controller (`#239 <https://github.com/ros-controls/ros2_controllers/issues/239>`_)
  * Add velocity pid support.
  * Remove incorrect init test for only velocity command interface.
  * Add clarification comments for pid aux variables. Adapt update loop.
  * Change dt for pid to appropriate measure.
  * Improve partial commands for velocity-only mode.
  * Extend tests to use velocity-only mode.
  * Increase timeout for velocity-only mode parametrized tests.
  * add is_same_sign for better refactor
  * refactor boolean logic
  * set velocity to 0.0 on deactivate
* Contributors: Lovro Ivanov, Bence Magyar

1.3.0 (2022-01-11)
------------------

1.2.0 (2021-12-29)
------------------

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
