:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/mecanum_drive_controller/doc/userdoc.rst

.. _mecanum_drive_controller_userdoc:

mecanum_drive_controller
=========================

Library with shared functionalities for mobile robot controllers with mecanum drive (four mecanum wheels).
The library implements generic odometry and update methods and defines the main interfaces.

Execution logic of the controller
----------------------------------

The controller uses velocity input, i.e., stamped Twist messages where linear ``x``, ``y``, and angular ``z`` components are used.
Values in other components are ignored.
In the chain mode, the controller provides three reference interfaces, one for linear velocity and one for steering angle position.
Other relevant features are:

  - odometry publishing as Odometry and TF message;
  - input command timeout based on a parameter.

Note about odometry calculation:
In the DiffDRiveController, the velocity is filtered out, but we prefer to return it raw and let the user perform post-processing at will.
We prefer this way of doing so as filtering introduces delay (which makes it difficult to interpret and compare behavior curves).


Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
When controller is in chained mode, it exposes the following references which can be commanded by the preceding controller:

- ``<controller_name>/linear/x/velocity``, in m/s
- ``<controller_name>/linear/y/velocity``, in m/s
- ``<controller_name>/angular/z/velocity``, in rad/s

Commands
,,,,,,,,,
- ``<*_wheel_command_joint_name>/velocity``, in rad/s

States
,,,,,,,
- ``<joint_name>/velocity``, in rad/s

.. note::

  ``joint_name`` can be of ``*_wheel_state_joint_name`` parameter (if used), ``*_wheel_command_joint_name`` otherwise.


Subscribers
,,,,,,,,,,,,
Used when the controller is not in chained mode (``in_chained_mode == false``).

- ``<controller_name>/reference``  [``geometry_msgs/msg/TwistStamped``]
  Velocity command for the controller, if ``use_stamped_vel == true``.

- ``<controller_name>/reference_unstamped``  [``geometry_msgs/msg/Twist``]
  Velocity command for the controller, if ``use_stamped_vel == false``.

Publishers
,,,,,,,,,,,
- ``<controller_name>/odometry``          [``nav_msgs/msg/Odometry``]
- ``<controller_name>/tf_odometry``       [``tf2_msgs/msg/TFMessage``]
- ``<controller_name>/controller_state``  [``control_msgs/msg/MecanumDriveControllerState``]

Parameters
,,,,,,,,,,,


This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/mecanum_drive_controller/src/mecanum_drive_controller.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/mecanum_drive_controller.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/mecanum_drive_controller/test/mecanum_drive_controller_params.yaml>`_:

.. literalinclude:: ../test/mecanum_drive_controller_params.yaml
   :language: yaml
