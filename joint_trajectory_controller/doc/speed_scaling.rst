Speed scaling
=============

The ``joint_trajectory_controller`` (JTC) supports dynamically scaling its trajectory execution speed.
That means, when specifying a scaling factor :math:`{f}` of less than 1, execution will proceed only
:math:`{f \cdot \Delta_t}` per control step where :math:`{\Delta_t}` is the controller's cycle time.

Methods of speed scaling
------------------------

Generally, the speed scaling feature has two separate scaling approaches in mind: On-Robot scaling
and On-Controller scaling. They are both conceptually different and to correctly configure speed
scaling it is important to understand the differences.

On-Robot speed scaling
~~~~~~~~~~~~~~~~~~~~~~

This scaling method is intended for robots that provide a scaling feature directly on the robot's
teach pendant and / or through a safety feature. One example of such robots are the `Universal
Robots manipulators <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>`_.

For the scope of this documentation a user-defined scaling and a safety-limited scaling will be
treated the same resulting in a "hardware scaling factor".

In this setup, the hardware will treat the command sent from the ROS controller (e.g. Reach joint
configuration :math:`{\theta}` within :math:`{\Delta_t}` seconds.). This effectively means that the
robot will only make half of the way towards the target configuration when a scaling factor of 0.5
is given (neglectling acceleration and deceleration influcences during this time period).

The following plot shows trajectory execution (for one joint) with a hardware-scaled execution and
a controller that is **not** aware of speed scaling:

.. image:: traj_without_speed_scaling.png
   :alt: Trajectory with a hardware-scaled-down execution with a non-scaled controller

The graph shows a trajectory with one joint being moved to a target point and back to its starting
point. As the joint's speed is limited to a very low setting on the teach pendant, speed scaling
(black line) activates and limits the joint speed (green line). As a result, the target trajectory
(light blue) doesn't get executed by the robot, but instead the pink trajectory is executed. The
vertical distance between the light blue line and the pink line is the path error in each control
cycle. We can see that the path deviation gets above 300 degrees at some point and the target point
at -6 radians never gets reached.

With the scaled version of the trajectory controller the example motion shown in the previous diagram becomes:

.. image:: traj_with_speed_scaling.png
   :alt: Trajectory with a hardware-scaled-down execution with a scaled controller

The deviation between trajectory interpolation on the ROS side and actual robot execution stays
minimal and the robot reaches the intermediate setpoint instead of returning "too early" as in the
example above.

.. todo:: Describe method behind this scaling approach.


On-Controller speed scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Conceptionally, with this scaling the robot hardware isn't aware of any scaling happening. The JTC
generates commands to be sent to the robot that are already scaled down accordingly, so they can be
directly executed by the robot.

Since the hardware isn't aware of speed scaling, the speed-scaling related command and state
interfaces should not be specified and the scaling factor will be set through the
``~/speed_scaling_input`` topic directly:

.. code:: console

   $ ros2 topic pub --qos-durability transient_local --once \
     /joint_trajectory_controller/speed_scaling_input control_msgs/msg/SpeedScalingFactor "{factor: 0.5}"

.. note::
   The ``~/speed_scaling_input`` topic uses the QoS durability profile ``transient_local``. This
   means you can restart the controller while still having a publisher on that topic active.

.. note::
   The current implementation only works for position-based interfaces.


Goal time tolerances
--------------------

.. todo::
   What happens to goal time tolerances if we scale down a trajectory?
