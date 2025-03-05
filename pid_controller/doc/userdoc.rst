:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/pid_controller/doc/userdoc.rst

.. _pid_controller_userdoc:

PID Controller
--------------------------------

PID Controller implementation that uses PidROS implementation from `control_toolbox <https://github.com/ros-controls/control_toolbox/>`_ package.
The controller can be used directly by sending references through a topic or in a chain having preceding or following controllers.
It also enables to use the first derivative of the reference and its feedback to have second-order PID control.

Depending on the reference/state and command interface of the hardware a different parameter setup of PidROS should be used as for example:

- reference/state POSITION; command VELOCITY --> PI CONTROLLER
- reference/state VELOCITY; command ACCELERATION --> PI CONTROLLER

- reference/state VELOCITY; command POSITION --> PD CONTROLLER
- reference/state ACCELERATION; command VELOCITY --> PD CONTROLLER

- reference/state POSITION; command POSITION --> PID CONTROLLER
- reference/state VELOCITY; command VELOCITY --> PID CONTROLLER
- reference/state ACCELERATION; command ACCELERATION --> PID CONTROLLER
- reference/state EFFORT; command EFFORT --> PID CONTROLLER

.. note::

   Theoretically one can misuse :ref:`Joint Trajectory Controller (JTC)<joint_trajectory_controller_userdoc>` for the same purpose by sending only one reference point into it.
   Nevertheless, this is not recommended. JTC should be used if you need to interpolate between trajectory points using linear, cubic or quintic interpolation. PID Controller doesn't do that.
   PID term of JTC has different purpose - it enables commanding only ``velocity`` or ``effort`` interfaces to hardware.

Execution logic of the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The controller can be also used in "feed-forward" mode where feed-forward gain is used to increase controllers dynamics.
If one type of the reference and state interfaces is used, only immediate error is used. If there are two, then the second interface type is considered to be the first derivative of the first type.
For example a valid combination would be ``position`` and ``velocity`` interface types.

Using the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pluginlib-Library: pid_controller
Plugin name: pid_controller/PidController

Description of controller's interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
- <reference_and_state_dof_names[i]>/<reference_and_state_interfaces[j]>  [double]
  **NOTE**: ``reference_and_state_dof_names[i]`` can be from ``reference_and_state_dof_names`` parameter, or if it is empty then ``dof_names``.

Commands
,,,,,,,,,
- <dof_names[i]>/<command_interface>  [double]

States
,,,,,,,
- <reference_and_state_dof_names[i]>/<reference_and_state_interfaces[j]>  [double]
  **NOTE**: ``reference_and_state_dof_names[i]`` can be from ``reference_and_state_dof_names`` parameter, or if it is empty then ``dof_names``.


Subscribers
,,,,,,,,,,,,
If controller is not in chained mode (``in_chained_mode == false``):

- <controller_name>/reference  [control_msgs/msg/MultiDOFCommand]

If controller parameter ``use_external_measured_states`` is true:

- <controller_name>/measured_state  [control_msgs/msg/MultiDOFCommand]

Services
,,,,,,,,,,,

- <controller_name>/set_feedforward_control  [std_srvs/srv/SetBool]

.. warning::
   This service is being deprecated in favour of setting the ``feedforward_gain`` parameter to a non-zero value.

Publishers
,,,,,,,,,,,
- <controller_name>/controller_state  [control_msgs/msg/MultiDOFStateStamped]

Parameters
,,,,,,,,,,,

The PID controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

List of parameters
=========================
.. warning::
   The parameter ``enable_feedforward`` is being deprecated in favor of setting the ``feedforward_gain`` parameter to a non-zero value.
   This might cause different behavior if currently the ``feedforward_gain`` is set to a non-zero value and not activated.

.. generate_parameter_library_details:: ../src/pid_controller.yaml


An example parameter file
=========================


An example parameter file for this controller can be found in `the test folder (standalone) <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/pid_controller/test/pid_controller_params.yaml>`_:

.. literalinclude:: ../test/pid_controller_params.yaml
   :language: yaml

or as `preceding controller <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/pid_controller/test/pid_controller_preceding_params.yaml>`_:

.. literalinclude:: ../test/pid_controller_preceding_params.yaml
   :language: yaml
