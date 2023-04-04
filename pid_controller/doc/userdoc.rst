.. _pid_controller_userdoc:

pid_controller
=========================

PID Controller implementation that uses PidROS implemenation from `control_toolbox <https://github.com/ros-controls/control_toolbox/>`_ package.
The controller can be used directly by sending references through a topic or in a chain having preceding or following controllers.
It also enables to use first derivation of the reference and its feedback to have second-order PID control.

Depending on the referece/state and command interface of hardware differnt parameters setup of PidROS should be used as for example:

- reference/state POSITION; command VELOCITY --> PI CONTROLLER
- reference/state VELOCITY; command ACCELERATION --> PI CONTROLLER

- reference/state VELOCITY; command POSITION --> PD CONTROLLER
- reference/state ACCELERATION; command VELOCITY --> PD CONTROLLER

- reference/state POSITION; command POSITION --> PID CONTROLLER
- reference/state VELOCITY; command VELOCITY --> PID CONTROLLER
- reference/state ACCELERATION; command ACCELERATION --> PID CONTROLLER
- reference/state EFFORT; command EFFORT --> PID CONTROLLER

.. note::

   Theretically one can missuse :ref:`Joint Trajectory Controller (JTC)<joint_trajectory_controller_userdoc>` for the same purpose by sending only one reference point into it.
   Nevertheless, this is not recommended. JTC should be use if you need to interpolate between trajectory points using linear, cubic or quintic interplation. PID Controller doesn't to that.
   PID term of JTC has differnet purpose - it enable commanding only ``velocity`` or ``effort`` interfaces to hardware. 

Execution logic of the controller
----------------------------------

The controller can be also used in "feed-forward" mode where feed-forward gain is used to increase controllers dynamics.
If one type of the reference and state interfaces is used only immediate error is used, if there are two, then the second interface type is considerd to be first derivative of the first type.
For example a valid combination would be ``position`` and ``velocity`` interface types.


Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
- <reference_and_state_dof_names[i]>/<reference_and_state_interfaces[j]>  [double]
  **NOTE**: ``reference_and_state_dof_names`` can be ``reference_and_state_dof_names`` parameter of if empty ``dof_names``.

Commands
,,,,,,,,,
- <dof_names[i]>/<command_interface>  [double]

States
,,,,,,,
- <reference_and_state_dof_names[i]>/<reference_and_state_interfaces[j]>  [double]
  **NOTE**: ``reference_and_state_dof_names`` can be ``reference_and_state_dof_names`` parameter of if empty ``dof_names``.


Subscribers
,,,,,,,,,,,,
Used when controller is not in chained mode (``in_chained_mode == false``).

- <controller_name>/reference  [control_msgs/msg/MultiDOFCommand]

Used when controller parameter ``use_external_measured_states`` is used.

- <controller_name>/measured_state  [control_msgs/msg/MultiDOFCommand]

Services
`````````

- <controller_name>/set_feedforward_control  [std_srvs/srv/SetBool]

Publishers
,,,,,,,,,,,
- <controller_name>/controller_state  [control_msgs/msg/MultiDOFStateStamped]

Parameters
,,,,,,,,,,,

For list of parameters and their meaning YAML file in the ``src`` folder of the controller's package.

For an exameplary parameterization see the ``test`` folder of the controller's package.
