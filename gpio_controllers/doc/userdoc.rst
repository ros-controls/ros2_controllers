:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gpio_controllers/doc/userdoc.rst

.. _gpio_controllers_userdoc:

gpio_controllers
=====================

This is a collection of controllers for hardware interfaces of type GPIO (``<gpio>`` tag in the URDF).

gpio_command_controller
-----------------------------
gpio_command_controller let the user expose command interfaces of given GPIO interfaces and publishes state interfaces of the configured command interfaces.

Description of controller's interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- ``/<controller_name>/gpio_states`` [``control_msgs/msg/DynamicJointState``]: Publishes all state interfaces of the given GPIO interfaces.
- ``/<controller_name>/commands`` [``control_msgs/msg/DynamicJointState``]:  A subscriber for configured command interfaces.


Parameters
^^^^^^^^^^^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gpio_controllers/src/gpio_command_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details::
  ../src/gpio_command_controller_parameters.yaml

The controller expects at least one GPIO interface and the corresponding command interface names or state interface. However, these Command and State interfaces are optional. The controller behaves as a broadcaster when no Command Interface is present, thereby publishing the configured GPIO state interfaces if set, else the one present in the URDF.

.. note::

  When no state interface is provided in the param file, the controller will try to use state_interfaces from ros2_control's config placed in the URDF for configured gpio interfaces.
  However, command interfaces will not be configured based on the available URDF setup.

.. code-block:: yaml

    gpio_command_controller:
      ros__parameters:
        type: gpio_controllers/GpioCommandController
        gpios:
          - Gpio1
          - Gpio2
        command_interfaces:
          Gpio1:
            - interfaces:
              - dig.1
              - dig.2
              - dig.3
          Gpio2:
            - interfaces:
              - ana.1
              - ana.2
        state_interfaces:
          Gpio2:
            - interfaces:
              - ana.1
              - ana.2

With the above-defined controller configuration, the controller will accept commands for all gpios' interfaces and will only publish the state of Gpio2.


gpio_tool_controller
-----------------------------

Controller for IO-based tools connected to a robot via general-purpose inputs and outputs (GPIO), digital or analog.
It is designed for industrial grippers, pneumatic clamps, lifts, brakes, and any component whose state is fully described by a set of GPIO signals — including virtual IOs driven by a PLC state machine.

The controller exposes a high-level *engage* / *disengage* interface and an optional *reconfigure* interface.
All transitions are executed through a deterministic six-step state machine that sets commands, waits for hardware confirmation, and handles timeouts and errors safely.
Reconfiguration (switching between physical tool configurations, e.g., jaw width) is only permitted in the *disengaged* state.

Pluginlib-Library: gpio_controllers

Plugin name: gpio_tool_controller/GpioToolController

State machine
^^^^^^^^^^^^^

Every engage, disengage, or reconfigure action is executed as an ordered sequence of steps.
The current step is tracked in the ``GPIOToolTransition`` enum and published on ``~/controller_state``.

.. code-block:: text

   ┌──────────────────────────────────────────────────────────┐
   │                       IDLE                               │
   │  (continuously verifies current state in the background) │
   └────────────────┬─────────────────────────────────────────┘
                    │  engage / disengage / reconfigure request
                    ▼
          SET_BEFORE_COMMAND   ← set preparatory commands (e.g., release brake)
                    │
                    ▼
         CHECK_BEFORE_COMMAND  ← wait for preparatory state confirmation
                    │
                    ▼
              SET_COMMAND      ← set main action commands (e.g., open/close valve)
                    │
                    ▼
             CHECK_COMMAND     ← wait for hardware to confirm new state
                    │
                    ▼
          SET_AFTER_COMMAND    ← set post-action commands (e.g., re-engage brake)
                    │
                    ▼
         CHECK_AFTER_COMMAND   ─── success ──► IDLE
                    │
                    └─────────── timeout ─────► HALTED

If any ``CHECK_*`` step does not reach the expected state within ``timeout`` seconds, or if the action is canceled, the controller transitions to **HALTED**.
In HALTED, all commands are frozen and no new requests are accepted until the operator calls the ``~/reset_halted`` service.

For the *engage* direction the final state is not known in advance (e.g., a gripper may land in ``close_empty`` or ``close_full`` depending on whether a part was detected).
The controller therefore accepts any state listed in ``possible_engaged_states`` as a valid result of CHECK_COMMAND.

High-level actions
^^^^^^^^^^^^^^^^^^

Two high-level actions drive the state machine:

- **Disengage** — moves the tool to its disengaged state (e.g., open gripper, lower lift).
- **Engage** — moves the tool to one of its possible engaged states (e.g., close gripper, raise lift).

If a new request arrives while an action is already running, the opposite action overrides it (e.g., a disengage request while engaging).
A request for the same action that is already running is rejected.
Reconfigure requests are rejected if the tool is not idle and disengaged.

.. note::

   The controller can be driven either through **services** (``use_action: false``, the default) or through **actions** (``use_action: true``).
   Services block the caller until the transition completes or fails.
   Actions provide non-blocking access with feedback on the current transition step.
   The ``~/reset_halted`` service is always available regardless of the mode.


ROS 2 interface of the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Publishers
,,,,,,,,,,

``/joint_states`` [``sensor_msgs/msg/JointState``]
  Joint positions for all joints listed in ``engaged_joints`` and ``configuration_joints``.
  Published every control cycle.
  Only created when at least one joint is configured.

``~/dynamic_interfaces`` [``control_msgs/msg/DynamicInterfaceValues``]
  Real-time snapshot of every command and state interface value used by the controller.
  Useful for monitoring and diagnostics.

``~/controller_state`` [``control_msgs/msg/GPIOToolControllerState``]
  Current tool state name, current configuration name, and the active transition step.
  Published every control cycle.

Services (``use_action: false``)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

``~/<disengaged.name>`` [``std_srvs/srv/Trigger``]
  Trigger a disengage action.
  Blocks until the transition completes.
  Returns ``success=false`` if the transition times out or is halted.
  Default service name: ``~/disengaged``.

``~/<engaged.name>`` [``std_srvs/srv/Trigger``]
  Trigger an engage action.
  Blocks until the transition completes.
  Returns ``success=false`` if the transition times out or is halted.
  Default service name: ``~/engaged``.

``~/reconfigure`` [``control_msgs/srv/SetGPIOToolConfig``]
  Switch to the named configuration.
  Only available when ``configurations`` is non-empty.
  Blocks until the transition completes.
  Returns ``success=false`` if the tool is not in the disengaged state, if the configuration name is unknown, or if the transition is halted.

``~/reset_halted`` [``std_srvs/srv/Trigger``]
  Clear the HALTED state and return to IDLE.
  After reset the controller checks the current hardware state and sets the tool state accordingly.
  Always available, regardless of ``use_action``.

Actions (``use_action: true``)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

``~/<engaged.name>_<disengaged.name>`` [``control_msgs/action/GPIOToolCommand``]
  Engage or disengage the tool.
  Set ``goal.engage = true`` to engage or ``goal.engage = false`` to disengage.
  Publishes feedback with the current transition step while the action is running.
  Returns ``success=false`` and aborts if the transition is halted.
  Default action name: ``~/engaged_disengaged``.

``~/reconfigure`` [``control_msgs/action/SetGPIOToolConfig``]
  Switch to the named configuration.
  Only available when ``configurations`` is non-empty.
  Returns ``success=false`` and aborts if the transition is halted.

``~/reset_halted`` [``std_srvs/srv/Trigger``]
  Clear the HALTED state.
  Always a service, even when ``use_action: true``.

ros2_control interfaces
^^^^^^^^^^^^^^^^^^^^^^^

Command interfaces
,,,,,,,,,,,,,,,,,,

All command interface names listed in the parameter file are claimed individually.
The full set is the union of all ``*.command.interfaces``, ``*.set_before_command.interfaces``, ``*.set_after_command.*_interfaces``, and ``configuration_setup.*.command_interfaces``.

State interfaces
,,,,,,,,,,,,,,,,,

All state interface names listed in the parameter file are claimed individually.
The full set is the union of all ``*.state.interfaces``, ``*.set_before_state.interfaces``, ``*.set_after_state.interfaces``, ``configuration_setup.*.state_interfaces``, and ``sensors_interfaces.*`` entries.

Parameters
^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The `parameter definition file in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gpio_controllers/src/gpio_tool_controller.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details::
  ../src/gpio_tool_controller.yaml

Example configurations
^^^^^^^^^^^^^^^^^^^^^^

The following examples illustrate common use cases.
All interface names refer to GPIO interfaces defined in the ``<ros2_control>`` block of the robot URDF.

Pneumatic gripper with two jaw states and two object-size configurations
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

A pneumatic gripper with a spring-loaded brake.
Opening (disengaging) requires the brake to be released first (``set_before_command``), then activating the open valve.
After reaching the open state the brake is re-engaged (``set_after_command``).
Closing can result in either ``close_empty`` (no part grasped) or ``close_full`` (part grasped), distinguished by the ``Part_Grasped_signal``.
The jaw separation is controlled by a separate configuration mechanism with two positions: ``narrow_objects`` and ``wide_objects``.

.. literalinclude:: ../test/gpio_tool_controller/test_gpio_tool_controller_gripper_example.yaml
   :language: yaml

Lift with payload detection
,,,,,,,,,,,,,,,,,,,,,,,,,,,

A simple pneumatic lift with no brake.
Raising (engaging) can end in ``high_empty`` or ``high_payload`` depending on whether a payload was detected.
No ``set_before_command`` or ``set_after_command`` steps are needed.

.. literalinclude:: ../test/gpio_tool_controller/test_gpio_tool_controller_lift_example.yaml
   :language: yaml
