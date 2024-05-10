:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes/Jazzy.rst

Iron to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Iron (previous) and Jazzy (current) releases.

admittance_controller
************************
* Remove ``robot_description`` parameter from parameter YAML, because it is not used at all (`#963 <https://github.com/ros-controls/ros2_controllers/pull/963>`_).

diff_drive_controller
*****************************
* The twist message on ``~/cmd_vel`` is now required to be of stamped type (`#812 <https://github.com/ros-controls/ros2_controllers/pull/812>`_).
* Remove unused parameter ``wheels_per_side`` (`#958 <https://github.com/ros-controls/ros2_controllers/pull/958>`_).

joint_trajectory_controller
*****************************

* Parameter ``allow_nonzero_velocity_at_trajectory_end`` is now per default ``false`` (`#834 <https://github.com/ros-controls/ros2_controllers/pull/834>`_).
* Activate update of dynamic parameters (`#761 <https://github.com/ros-controls/ros2_controllers/pull/761>`_ and `#849 <https://github.com/ros-controls/ros2_controllers/pull/849>`_).
* The parameter ``start_with_holding`` is removed, it now always holds the position at activation (`#839 <https://github.com/ros-controls/ros2_controllers/pull/839>`_).
* Continue with last trajectory-point on success, instead of hold-position from current state (`#842 <https://github.com/ros-controls/ros2_controllers/pull/842>`_).
* Add console output for tolerance checks (`#932 <https://github.com/ros-controls/ros2_controllers/pull/932>`_):

  .. code::

    [tolerances]: State tolerances failed for joint 2:
    [tolerances]: Position Error: 0.020046, Position Tolerance: 0.010000
    [trajectory_controllers]: Aborted due goal_time_tolerance exceeding by 1.010000 seconds

* Goals are now cancelled in ``on_deactivate`` transition (`#962 <https://github.com/ros-controls/ros2_controllers/pull/962>`_).
* Empty trajectory messages are discarded (`#902 <https://github.com/ros-controls/ros2_controllers/pull/902>`_).
* Action field ``error_string`` is now filled with meaningful strings (`#887 <https://github.com/ros-controls/ros2_controllers/pull/887>`_).
* Angle wraparound behavior (continuous joints) was added from the current state to the first segment of the incoming trajectory (`#796 <https://github.com/ros-controls/ros2_controllers/pull/796>`_).
* The URDF is now parsed for continuous joints and angle wraparound is automatically activated now (`#949 <https://github.com/ros-controls/ros2_controllers/pull/949>`_).

pid_controller
************************
* 🚀 The PID controller was added 🎉 (`#434 <https://github.com/ros-controls/ros2_controllers/pull/434>`_).

steering_controllers_library
********************************
* Changing default int values to double in steering controller's yaml file. The controllers should now initialize successfully without specifying these parameters (`#927 <https://github.com/ros-controls/ros2_controllers/pull/927>`_).
* A fix for Ackermann steering odometry was added (`#921 <https://github.com/ros-controls/ros2_controllers/pull/921>`_).

tricycle_controller
************************
* tricycle_controller now uses generate_parameter_library (`#957 <https://github.com/ros-controls/ros2_controllers/pull/957>`_).
