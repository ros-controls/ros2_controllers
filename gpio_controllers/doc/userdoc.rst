.. _gpio_controllers_userdoc:

gpio_controllers
=====================

This is a collection of controllers for gpios that work with multiple interfaces.

Hardware interface type
-----------------------

These controllers work with gpios using user defined command interfaces.

Using GPIO Command Controller
-----------------------------
The controller expects at least one gpio interface abd the corresponding command interface names.
A yaml file for using it could be:
.. code-block:: yaml

    controller_manager:
        ros__parameters:
            update_rate: 100  # Hz
            joint_state_broadcaster:
            type: joint_state_broadcaster/JointStateBroadcaster
            gpio_command_controller:
            type: gpio_controllers/GpioCommandController

    gpio_command_controller:
        ros__parameters:
            gpios:
                - Gpio1
                - Gpio2
            command_interfaces:
                Gpio1:
                    - ports:
                        - dig.1
                        - dig.2
                        - dig.3
                Gpio2:
                    -ports:
                        - ana.1
                        - ana.2
