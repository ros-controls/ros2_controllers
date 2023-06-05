:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/steering_controllers_library/doc/userdoc.rst

.. _steering_controllers_library_userdoc:

steering_controllers_library
=============================

Library with shared functionalities for mobile robot controllers with steering drive (2 degrees of freedom).
The library implements generic odometry and update methods and defines the main interfaces.

Nomenclature used for the controller is used from `wikipedia <https://en.wikipedia.org/wiki/Wheelbase>`_.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped or unstamped Twist messages where linear ``x`` and angular ``z`` components are used.
Angular component under
Values in other components are ignored.
In the chain mode the controller provides two reference interfaces, one for linear velocity and one for steering angle position.
Other relevant features are:

* support for front and rear steering configurations;
* odometry publishing as Odometry and TF message;
* input command timeout based on a parameter.

The command for the wheels are calculated using ``odometry`` library where based on concrete kinematics traction and steering commands are calculated.
Currently implemented kinematics in corresponding packages are:

* :ref:`Bicycle <bicycle_steering_controller_userdoc>` - with one steering and one drive joints;
* :ref:`Tricylce <tricycle_steering_controller_userdoc>` - with one steering and two drive joints;
* :ref:`Ackermann <ackermann_steering_controller_userdoc>` - with two seering and two drive joints.



Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
- <controller_name>/linear/velocity      [double], in m/s
- <controller_name>/angular/position     [double]  # in [rad]

Commands
,,,,,,,,,
``front_steering == true``

- <front_wheels_names[i]>/position     [double]  # in [rad]
- <rear_wheels_names[i]>/velocity      [double]  # in [m/s]

``front_steering == false``

- <front_wheels_names[i]>/velocity     [double]  # in [m/s]
- <rear_wheels_names[i]>/position      [double]  # in [rad]

States
,,,,,,,
Depending on the ``position_feedback``, different feedback types are expected

* ``position_feedback == true`` --> ``TRACTION_FEEDBACK_TYPE = position``
* ``position_feedback == false`` --> ``TRACTION_FEEDBACK_TYPE = velocity``

``front_steering == true``

- <front_wheels_names[i]>/position                  [double]  # in [rad]
- <rear_wheels_names[i]>/<TRACTION_FEEDBACK_TYPE>   [double]  # in [m] or [m/s]

``front_steering == false``

- <front_wheels_names[i]>/<TRACTION_FEEDBACK_TYPE>  [double]  # [m] or [m/s]
- <rear_wheels_names[i]>/position                   [double]  # in [rad]

Subscribers
,,,,,,,,,,,,
Used when controller is not in chained mode (``in_chained_mode == false``).

- <controller_name>/reference  [geometry_msgs/msg/TwistStamped]
  If parameter ``use_stamped_vel`` is ``true``.
- <controller_name>/reference_unstamped   [geometry_msgs/msg/Twist]
  If parameter ``use_stamped_vel`` is ``false``.

Publishers
,,,,,,,,,,,
- <controller_name>/odometry          [nav_msgs/msg/Odometry]
- <controller_name>/tf_odometry       [tf2_msgs/msg/TFMessage]
- <controller_name>/controller_state  [control_msgs/msg/SteeringControllerStatus]

Parameters
,,,,,,,,,,,
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

.. generate_parameter_library_details:: ../src/steering_controllers_library.yaml
