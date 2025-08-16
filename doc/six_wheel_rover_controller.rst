.. _doc_six_wheel_rover_controller:

six_wheel_rover_controller
=============================

This page introduces the kinematic model for a six-wheeled rover featuring four-wheel steering, commonly used in rocker-bogie style vehicles.

Kinematics of the system
------------------------

The controller uses a standard Ackermann-style kinematic model adapted for a six-wheel configuration. The core principle is that to avoid wheel slip during a turn, the axes of all wheels must intersect at a single point known as the **Instantaneous Center of Rotation (ICR)**.

The parameters shown in the diagram are essential for the kinematic calculations.

Diagram
------------------
.. image:: images/six_wheeler_steering.svg
   :width: 600px
   :align: center
   :alt: Six-wheel steering controller kinematic diagram

Inverse Kinematics
------------------

Inverse kinematics determine the required joint commands from a desired vehicle body velocity (a ``Twist`` message).

**Steering Joint Angles**

Given a desired turning radius, :math:`R`, the required steering angles for the inner and outer front wheels are calculated as follows:

.. math::

   \theta_{inner} = \arctan\left(\frac{d_3}{|R| - d_1}\right)

.. math::

   \theta_{outer} = \arctan\left(\frac{d_3}{|R| + d_1}\right)

The rear wheels are commanded to the same angle magnitudes but in the opposite direction to articulate the turn.

**Drive Wheel Velocities**

The velocity of each drive wheel is proportional to its distance from the ICR. The vehicle's angular velocity, :math:`\omega_{center}`, is first determined from the desired linear velocity, :math:`v`:

.. math::

   \omega_{center} = \frac{v}{R}

The linear velocity for each wheel (:math:`v_{wheel}`) is then found based on its unique turning radius. This is finally converted to the motor's angular velocity command, :math:`\omega_{wheel}`:

.. math::

   \omega_{wheel} = \frac{v_{wheel}}{r_{wheel}}

Forward Kinematics
------------------

Forward kinematics estimate the vehicle's current ``Twist`` from its joint states. The rover's linear velocity, :math:`v_x`, is estimated from the average velocity of the two non-steerable middle wheels.

.. math::

   v_x = \frac{(\omega_{left\_middle} + \omega_{right\_middle})}{2} \cdot r_{wheel}

The vehicle's angular velocity, :math:`\omega_z`, is then derived from this linear velocity and an estimation of the current turning radius, :math:`R_{approx}`.

.. math::

   \omega_z = \frac{v_x}{R_{approx}}

Controller Parameters
---------------------

.. list-table:: Controller Parameters
   :widths: 20 80
   :header-rows: 1

   * - Parameter
     - Description
   * - ``traction_joints``
     - A list of the six drive wheel joint names from the robot's URDF model, in the following order: front-left, front-right, middle-left, middle-right, rear-left, rear-right.
   * - ``steering_joints``
     - A list of the four corner steering joint names from the robot's URDF model, in the following order: front-left, front-right, rear-left, rear-right.
   * - ``d1``
     - Half the front/rear track width (meters). Lateral distance from the longitudinal center to the steering pivot.
   * - ``d2``
     - Longitudinal distance from rover center to the rear axle (meters).
   * - ``d3``
     - Longitudinal distance from rover center to the front axle (meters).
   * - ``d4``
     - Half the middle track width (meters). Lateral distance from the center to the middle wheel's center.
   * - ``wheel_radius``
     - Radius of the drive wheels (meters).
   * - ``odom_frame_id``
     - The name of the odometry frame. Default: ``odom``.
   * - ``base_frame_id``
     - The name of the robot's base frame. Default: ``base_link``.

Example Configuration
---------------------

.. code-block:: yaml

   six_wheel_steering_controller:
     ros__parameters:

       # ---- JOINT CONFIGURATION ----
       # [ACTION REQUIRED] Replace with the exact joint names from your robot's URDF.
       traction_joints: [
         "front_left_wheel_joint",
         "middle_left_wheel_joint",
         "rear_left_wheel_joint",
         "front_right_wheel_joint",
         "middle_right_wheel_joint",
         "rear_right_wheel_joint"
       ]
       steering_joints: [
         "front_left_steer_joint",
         "rear_left_steer_joint",
         "front_right_steer_joint",
         "rear_right_steer_joint"
       ]

       # ---- KINEMATIC PARAMETERS (in meters) ----
       # [ACTION REQUIRED] Measure these values from your specific rover.
       d1: 0.4
       d2: 0.5
       d3: 0.5
       d4: 0.45
       wheel_radius: 0.15

       # ---- ODOMETRY CONFIGURATION ----
       odom_frame_id: "odom"
       base_frame_id: "base_link"

