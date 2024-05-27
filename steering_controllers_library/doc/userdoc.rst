:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/steering_controllers_library/doc/userdoc.rst

.. _steering_controllers_library_userdoc:

steering_controllers_library
=============================

.. _steering_controller_status_msg: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/SteeringControllerStatus.msg
.. _odometry_msg: https://github.com/ros2/common_interfaces/blob/{DISTRO}/nav_msgs/msg/Odometry.msg
.. _twist_msg: https://github.com/ros2/common_interfaces/blob/{DISTRO}/geometry_msgs/msg/TwistStamped.msg
.. _tf_msg: https://github.com/ros2/geometry2/blob/{DISTRO}/tf2_msgs/msg/TFMessage.msg

Library with shared functionalities for mobile robot controllers with steering drives, with so-called non-holonomic constraints.

The library implements generic odometry and update methods and defines the main interfaces.

The update methods only use inverse kinematics, it does not implement any feedback control loops like path-tracking controllers etc.

For an introduction to mobile robot kinematics and the nomenclature used here, see :ref:`mobile_robot_kinematics`.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped `twist messages <twist_msg_>`_ where linear ``x`` and angular ``z`` components are used.
Values in other components are ignored.

In the chain mode the controller provides two reference interfaces, one for linear velocity and one for steering angle position.
Other relevant features are:

* support for front and rear steering configurations;
* odometry publishing as Odometry and TF message;
* input command timeout based on a parameter.

The command for the wheels are calculated using ``odometry`` library where based on concrete kinematics traction and steering commands are calculated.

Currently implemented kinematics
--------------------------------------------------------------

* :ref:`Bicycle <bicycle_steering_controller_userdoc>` - with one steering and one drive joints;
* :ref:`Tricylce <tricycle_steering_controller_userdoc>` - with one steering and two drive joints;
* :ref:`Ackermann <ackermann_steering_controller_userdoc>` - with two steering and two drive joints.

.. toctree::
  :hidden:

   Bicycle <../../bicycle_steering_controller/doc/userdoc.rst>
   Tricylce <../../tricycle_steering_controller/doc/userdoc.rst>
   Ackermann <../../ackermann_steering_controller/doc/userdoc.rst>

Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

Used when controller is in chained mode (``in_chained_mode == true``).

- ``<controller_name>/linear/velocity``      double, in m/s
- ``<controller_name>/angular/position``     double, in rad

Command interfaces
,,,,,,,,,,,,,,,,,,,

If parameter ``front_steering == true``

- ``<front_wheels_names[i]>/position``     double, in rad
- ``<rear_wheels_names[i]>/velocity``      double, in m/s

If parameter ``front_steering == false``

- ``<front_wheels_names[i]>/velocity``     double, in m/s
- ``<rear_wheels_names[i]>/position``      double, in rad

State interfaces
,,,,,,,,,,,,,,,,,

Depending on the ``position_feedback``, different feedback types are expected

* ``position_feedback == true`` --> ``TRACTION_FEEDBACK_TYPE = position``
* ``position_feedback == false`` --> ``TRACTION_FEEDBACK_TYPE = velocity``

If parameter ``front_steering == true``

- ``<front_wheels_names[i]>/position``                  double, in rad
- ``<rear_wheels_names[i]>/<TRACTION_FEEDBACK_TYPE>``   double, in m or m/s

If parameter ``front_steering == false``

- ``<front_wheels_names[i]>/<TRACTION_FEEDBACK_TYPE>``  double, in m or m/s
- ``<rear_wheels_names[i]>/position``                   double, in rad

Subscribers
,,,,,,,,,,,,

Used when controller is not in chained mode (``in_chained_mode == false``).

- ``<controller_name>/reference``  [`geometry_msgs/msg/TwistStamped <twist_msg_>`_]

Publishers
,,,,,,,,,,,

- ``<controller_name>/odometry``          [`nav_msgs/msg/Odometry <odometry_msg_>`_]
- ``<controller_name>/tf_odometry``       [`tf2_msgs/msg/TFMessage <tf_msg_>`_]
- ``<controller_name>/controller_state``  [`control_msgs/msg/SteeringControllerStatus <steering_controller_status_msg_>`_]

Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

.. generate_parameter_library_details:: ../src/steering_controllers_library.yaml
