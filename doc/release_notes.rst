:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Galactic to Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Galactic (previous) and Humble (current) releases. Bugfixes are not included in this list.

.. note::

  This list was created in July 2024, earlier changes may not be included.

diff_drive_controller
*****************************
* Remove unused parameter ``wheels_per_side`` (`#958 <https://github.com/ros-controls/ros2_controllers/pull/958>`_).

joint_trajectory_controller
*****************************

* Activate update of dynamic parameters (`#761 <https://github.com/ros-controls/ros2_controllers/pull/761>`_ and `#849 <https://github.com/ros-controls/ros2_controllers/pull/849>`_).
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
* Tolerances sent with the action goal are now processed and used for the action. (`#716 <https://github.com/ros-controls/ros2_controllers/pull/716>`_). For details, see the `JointTolerance message <https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/JointTolerance.msg>`_:

  .. code-block:: markdown

    The tolerances specify the amount the position, velocity, and
    accelerations can vary from the setpoints.  For example, in the case
    of trajectory control, when the actual position varies beyond
    (desired position + position tolerance), the trajectory goal may
    abort.

    There are two special values for tolerances:
     * 0 - The tolerance is unspecified and will remain at whatever the default is
     * -1 - The tolerance is "erased".  If there was a default, the joint will be
            allowed to move without restriction.

pid_controller
************************
* 🚀 The PID controller was added 🎉 (`#434 <https://github.com/ros-controls/ros2_controllers/pull/434>`_).

steering_controllers_library
********************************
* Changing default int values to double in steering controller's yaml file. The controllers should now initialize successfully without specifying these parameters (`#927 <https://github.com/ros-controls/ros2_controllers/pull/927>`_).
