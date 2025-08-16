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



### gpio_tool_controller

The controller enables control of tools connected to the robot or components of a robot via general-purpose inputs and outputs (GPIO), digital or analog.
Concretely the controller is made for controlling the custom-made industrial grippers or tools, or components like lifts or breaks.
The IOs might even be virtual, like controlling the state machine of a robot that is implemented on a PLC.

The controller provides high-level interface for engaging or disengaging a tool or a component, as well as for reconfiguring it.
The reconfiguration is possible only in *disengaged* state.
In the example of gripper this would be *open* state.



The controller implements state machine for the transition between states and


1. Tool Actions

1. Tool state machine

2. Transitions
