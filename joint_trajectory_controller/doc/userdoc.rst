:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_trajectory_controller/doc/userdoc.rst

.. _joint_trajectory_controller_userdoc:

joint_trajectory_controller
===========================

Controller for executing joint-space trajectories on a group of joints.
The controller interpolates in time between the points so that their distance can be arbitrary.
Even trajectories with only one point are accepted.
Trajectories are specified as a set of waypoints to be reached at specific time instants,
which the controller attempts to execute as well as the mechanism allows.
Waypoints consist of positions, and optionally velocities and accelerations.

*Parts of this documentation were originally published in the ROS 1 wiki under the* `CC BY 3.0 license <https://creativecommons.org/licenses/by/3.0/>`_. *Citations are given in the respective section, but were adapted for the ROS 2 implementation.* [#f1]_

Hardware interface type [#f1]_
-------------------------------

Currently joints with position, velocity, acceleration, and effort interfaces are supported. The joints can have one or more command interfaces, where the following control laws are applied at the same time:

* For command interfaces ``position``, the desired positions are simply forwarded to the joints,
* For command interfaces ``acceleration``, desired accelerations are simply forwarded to the joints.
* For ``velocity`` (``effort``) command interfaces, the position+velocity trajectory following error is mapped to ``velocity`` (``effort``) commands through a PID loop (:ref:`parameters`).

This leads to the the following allowed combinations of command and state interfaces:

* With command interface ``position``, there are no restrictions for state interfaces.
* With command interface ``velocity``:

  * if command interface ``velocity`` is the only one, state interfaces must include  ``position, velocity`` .
  * no restrictions otherwise.

* With command interface ``effort``, state interfaces must include  ``position, velocity``.
* With command interface ``acceleration``, there are no restrictions for state interfaces.

Example controller configurations can be found :ref:`below <ROS 2 interface>`.

Similarly to the trajectory representation case above, it's possible to support new hardware interfaces, or alternative mappings to an already supported interface (eg. a proxy controller for generating effort commands).

Other features
--------------

* Realtime-safe implementation.

* Proper handling of wrapping (continuous) joints.

* Robust to system clock changes: Discontinuous system clock changes do not cause discontinuities in the execution of already queued trajectory segments.


Using Joint Trajectory Controller(s)
------------------------------------

The controller expects at least position feedback from the hardware.
Joint velocities and accelerations are optional.
Currently the controller does not internally integrate velocity from acceleration and position from velocity.
Therefore if the hardware provides only acceleration or velocity states they have to be integrated in the hardware-interface implementation of velocity and position to use these controllers.

The generic version of Joint Trajectory controller is implemented in this package.
A yaml file for using it could be:

   .. code-block:: yaml

      controller_manager:
        ros__parameters:
          joint_trajectory_controller:
          type: "joint_trajectory_controller/JointTrajectoryController"

      joint_trajectory_controller:
        ros__parameters:
          joints:
            - joint1
            - joint2
            - joint3
            - joint4
            - joint5
            - joint6

          command_interfaces:
            - position

          state_interfaces:
            - position
            - velocity

          action_monitor_rate: 20.0

          allow_partial_joints_goal: false
          open_loop_control: true
          constraints:
            stopped_velocity_tolerance: 0.01
            goal_time: 0.0
            joint1:
              trajectory: 0.05
              goal: 0.03


Preemption policy [#f1]_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only one action goal can be active at any moment, or none if the topic interface is used. Path and goal tolerances are checked only for the trajectory segments of the active goal.

When an active action goal is preempted by another command coming from the action interface, the goal is canceled and the client is notified. The trajectory is replaced in a defined way, see :ref:`trajectory replacement <joint_trajectory_controller_trajectory_replacement>`.

Sending an empty trajectory message from the topic interface (not the action interface) will override the current action goal and not abort the action.


.. _ROS 2 interface:

Description of controller's interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

References
,,,,,,,,,,,,,,,,,,

(the controller is not yet implemented as chainable controller)

States
,,,,,,,,,,,,,,,,,,

The state interfaces are defined with ``joints`` and ``state_interfaces`` parameters as follows: ``<joint>/<state_interface>``.
Supported state interfaces are ``position``, ``velocity``, ``acceleration`` and ``effort`` as defined in the `hardware_interface/hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_.

Legal combinations of state interfaces are:

* ``position``
* ``position`` and ``velocity``
* ``position``, ``velocity`` and ``acceleration``
* ``effort``

Commands
,,,,,,,,,

There are two mechanisms for sending trajectories to the controller:

* via action, see :ref:`actions <Actions>`
* via topic, see :ref:`subscriber <Subscriber>`

Both use the ``trajectory_msgs/msg/JointTrajectory`` message to specify trajectories, and require specifying values for all the controller joints (as opposed to only a subset) if ``allow_partial_joints_goal`` is not set to ``True``. For further information on the message format, see :ref:`trajectory representation <joint_trajectory_controller_trajectory_representation>`.

.. _Actions:

Actions  [#f1]_
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

<controller_name>/follow_joint_trajectory [control_msgs::action::FollowJointTrajectory]
  Action server for commanding the controller


The primary way to send trajectories is through the action interface, and should be favored when execution monitoring is desired.
Action goals allow to specify not only the trajectory to execute, but also (optionally) path and goal tolerances.
When no tolerances are specified, the defaults given in the parameter interface are used (see :ref:`parameters`).
If tolerances are violated during trajectory execution, the action goal is aborted, the client is notified, and the current position is held.

.. _Subscriber:

Subscriber [#f1]_
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

<controller_name>/joint_trajectory [trajectory_msgs::msg::JointTrajectory]
  Topic for commanding the controller

The topic interface is a fire-and-forget alternative. Use this interface if you don't care about execution monitoring.
The controller's path and goal tolerance specification is not used in this case, as there is no mechanism to notify the sender about tolerance violations.
Note that although some degree of monitoring is available through the ``~/query_state`` service and ``~/state`` topic it is much more cumbersome to realize than with the action interface.


Publishers
,,,,,,,,,,,

<controller_name>/controller_state [control_msgs::msg::JointTrajectoryControllerState]
  Topic publishing internal states with the update-rate of the controller manager


Services
,,,,,,,,,,,

<controller_name>/query_state [control_msgs::srv::QueryTrajectoryState]
  Query controller state at any future time


Specialized versions of JointTrajectoryController
--------------------------------------------------------------
(TBD in ...)

The controller types are placed into namespaces according to their command types for the hardware (see :ref:`controllers`).

The following version of the Joint Trajectory Controller are available mapping the following interfaces:

* position_controllers::JointTrajectoryController


Further information
--------------------------------------------------------------

.. toctree::
   :titlesonly:

   Trajectory Representation <trajectory.rst>
   joint_trajectory_controller Parameters <parameters.rst>


.. rubric:: Footnote

.. [#f1] Adolfo Rodriguez: `joint_trajectory_controller <http://wiki.ros.org/joint_trajectory_controller>`_
