admittance_controller
==========================================

The admittance controller operates in one of two modes: chained and non-chained.
In the non-chained mode, the controller listens to joint commands on the `~/joint_commands` topic to get a reference. In chained mode, the admittance controller receives its reference from another controller. In both modes, the controller applies the calculated admittance values to the received reference and write the modified reference to its command interfaces.

The controller requires an external kinematics plugin to function. The [kinematics_interface](https://github.com/ros-controls/kinematics_interface) repository provides an interface and an implementation that the admittance controller can use.

## Parameters:
The admittance controller's uses the [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) to handle its parameters. The parameter definition file, located [here](../admittance_controller/src/admittance_controller_parameters.yaml) contains descriptions for all the parameters used by the controller.  
