.. _gpio_controllers_userdoc:

gpio_controllers
=====================

This is a collection of controllers for hardware interfaces of type GPIO (``<gpio>`` tag in the URDF).

gpio_command_controller
-----------------------------
gpio_command_controller publishes all state interfaces of given GPIO interfaces and let the user expose command interfaces.

Description of controller's interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- ``/<controller_name>/gpio_states`` [``control_msgs/msg/DynamicJointState``]: Publishes all state interfaces of the given GPIO interfaces.
- ``/<controller_name>/commands`` [``control_msgs/msg/DynamicJointState``]:  A subscriber for configured command interfaces.


Parameters
^^^^^^^^^^^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gpio_controllers_/src/gpio_command_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details::
  ../src/gpio_command_controller_parameters.yaml

The controller expects at least one gpio interface and the corresponding command interface names.
A yaml file for using it could be:

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
