:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/cartesian_trajectory_controller/doc/userdoc.rst

.. _cartesian_trajectory_controller_userdoc:

cartesian_trajectory_controller
===============================

Controller for executing Cartesian motion on a serial manipulator.
It accepts end-effector poses, interpolates between them in Cartesian space, and converts the result
into an ordinary joint trajectory using inverse kinematics.
The joint trajectory is then executed by the :ref:`joint_trajectory_controller
<joint_trajectory_controller_userdoc>`, which this controller derives from.

The difference to the joint trajectory controller is where the interpolation happens.
The joint trajectory controller interpolates between waypoints in *joint* space, so the tool center
point does not travel along a straight line between two poses, it bows off the line by an amount that
depends on the arm's configuration.
This controller interpolates in *Cartesian* space and solves the inverse kinematics for each
interpolated pose, so the tool center point stays on the commanded path.
That makes it suitable for the linear motions used in industrial applications, and for executing
end-effector pose targets produced by a planner or a policy.

The controller requires an external kinematics plugin.
The `kinematics_interface <https://github.com/ros-controls/kinematics_interface>`_ repository
provides the interface and a KDL based implementation, which is used by default.

How the controller builds a trajectory
--------------------------------------

Each incoming message is converted into a joint trajectory in four steps:

1. The current end-effector pose is prepended to the poses in the message, so the motion starts from
   where the robot currently is.
2. The translation is interpolated with a cubic spline through the resulting waypoints, and the
   orientation with spherical linear interpolation (SLERP) between consecutive quaternions.
   Quaternions are sign aligned first, so every segment rotates along the shorter arc.
3. The path is sampled every ``resample_dt`` seconds, and differential inverse kinematics is run at
   each sample to obtain the corresponding joint positions.
4. The resulting joint trajectory is handed to the joint trajectory controller, which executes it in
   the real-time loop.

The conversion runs once per message in the subscription callback, not in the control loop, so the
real-time path is unchanged from the joint trajectory controller.

Timing
^^^^^^

If the incoming poses carry ``time_from_start``, that timing is used as given.
This is the usual case when the poses come from a planner or from a policy that knows its own output
rate.

If the message carries no timing, the controller synthesizes it from ``max_cartesian_speed`` and
``max_angular_speed``.
Each segment is given the time needed to cover it without exceeding either limit, with a floor of
``resample_dt`` so that coincident poses still advance in time.

Choosing ``resample_dt``
^^^^^^^^^^^^^^^^^^^^^^^^

``resample_dt`` sets how finely the Cartesian path is sampled before the inverse kinematics runs.
Smaller values follow the commanded path more closely and cost more inverse kinematics steps per
message.
As a starting point, keep it below the spacing of the incoming poses, otherwise the controller
samples the path more coarsely than it was given and some of the commanded poses are passed over.

Using the Cartesian Trajectory Controller
-----------------------------------------

The controller expects at least position feedback from the hardware, and a robot description that
contains the chain between ``kinematics.base`` and ``kinematics.tip``.

Because the controller inherits from the joint trajectory controller, it takes that controller's
parameters as well as its own.
The ``joints``, ``command_interfaces`` and ``state_interfaces`` parameters are used exactly as
described in :ref:`its documentation <joint_trajectory_controller_userdoc>`.

.. code-block:: yaml

   cartesian_motion:
     ros__parameters:
       joints:
         - joint_1
         - joint_2
         - joint_3
         - joint_4
         - joint_5
         - joint_6
       command_interfaces:
         - position
       state_interfaces:
         - position

       kinematics:
         plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
         plugin_package: kinematics_interface
         base: base_link
         tip: tool0

       max_cartesian_speed: 0.1
       max_angular_speed: 0.5
       resample_dt: 0.01

.. note::
   The joint-space command inputs of the joint trajectory controller are disabled by this
   controller.
   The ``~/joint_trajectory`` topic and the ``~/follow_joint_trajectory`` action are not created, so
   the only way to command motion is the Cartesian reference topic described below.

Description of controller's interfaces
--------------------------------------

References
^^^^^^^^^^

(the controller is not yet implemented as chainable controller)

States
^^^^^^

The state interfaces are defined with the ``joints`` and ``state_interfaces`` parameters as follows:
``<joint>/<state_interface>``.
The controller reads joint positions to determine where the motion starts, so ``position`` has to be
among them.

Commands
^^^^^^^^

The command interfaces are defined with the ``joints`` and ``command_interfaces`` parameters as
follows: ``<joint>/<command_interface>``.
The controller produces a joint trajectory, so the legal combinations are the same as for the
:ref:`joint trajectory controller <joint_trajectory_controller_userdoc>`.

Subscriber
^^^^^^^^^^

<controller_name>/cartesian_reference [trajectory_msgs::msg::MultiDOFJointTrajectory]
  Topic for commanding the controller.

Each point carries one transform, which is the target pose of ``kinematics.tip`` expressed in
``kinematics.base``.
A message may hold a single pose or a sequence of them, so both a single target and a longer path
can be sent the same way.
The ``header.frame_id`` has to be either empty or equal to ``kinematics.base``, otherwise the message
is ignored.

The ``header.stamp`` is carried over to the generated joint trajectory, so the start time behaves as
it does for the joint trajectory controller.

Publishers
^^^^^^^^^^

<controller_name>/controller_state [control_msgs::msg::JointTrajectoryControllerState]
  Topic publishing the internal state of the underlying joint trajectory controller, with the
  update rate of the controller manager.

Parameters
----------

This controller uses the `generate_parameter_library
<https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.
The parameter `definition file located in the src folder
<https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/cartesian_trajectory_controller/src/cartesian_trajectory_controller_parameters.yaml>`_
contains descriptions for all the parameters used by the controller.
The parameters of the :ref:`joint trajectory controller <joint_trajectory_controller_userdoc>` apply
in addition to the ones listed here.

.. generate_parameter_library_details:: ../src/cartesian_trajectory_controller_parameters.yaml

An example parameter file for this controller can be found in `the test folder
<https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/cartesian_trajectory_controller/test/cartesian_trajectory_controller_params.yaml>`_:

.. literalinclude:: ../test/cartesian_trajectory_controller_params.yaml
   :language: yaml
