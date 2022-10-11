.. _joint_trajectory_controller_userdoc:

Admittance Controller
======================

Admittance controller enables you do zero-force control from a force measured on your TCP.
The controller implements ``ChainedControllerInterface``, so it is possible to add another controllers in front of it, e.g., ``JointTrajectoryController``.

The controller requires an external kinematics plugin to function. The `kinematics_interface <https://github.com/ros-controls/kinematics_interface>`_ repository provides an interface and an implementation that the admittance controller can use.

Parameters:



ROS2 interface of the controller
---------------------------------

Parameters
^^^^^^^^^^^

The admittance controller's uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.
The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/master/admittance_controller/src/admittance_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.
An example parameter file can be found in the `test folder of the controller <https://github.com/ros-controls/ros2_controllers/blob/master/admittance_controller/test/test_params.yaml>`_


Topics
^^^^^^^

~/joint_references (input topic) [trajectory_msgs::msg::JointTrajectoryPoint]
  Target joint commands when controller is not in chained mode.

~/state (output topic) [control_msgs::msg::AdmittanceControllerState]
  Topic publishing internal states.


ros2_control interfaces
------------------------

References
^^^^^^^^^^^
The controller has ``position`` and ``velocity`` reference interfaces exported in the format:
``<controller_name>/<joint_name>/[position|velocity]``


States
^^^^^^^
The state interfaces are defined with ``joints`` and ``state_interfaces`` parameters as follows: ``<joint>/<state_interface>``.
Supported state interfaces are ``position``, ``velocity``, and ``acceleration`` as defined in the `hardware_interface/hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_.
If some interface is not provided, the last commanded interface will be used for calculation.

For handling TCP wrenches `*Force Torque Sensor* semantic component  (from package *controller_interface*) <https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/semantic_components/force_torque_sensor.hpp>`_ is used.
The interfaces have prefix ``ft_sensor.name``, building the interfaces: ``<sensor_name>/[force.x|force.y|force.z|torque.x|torque.y|torque.z]``.


Commands
^^^^^^^^^
The command interfaces are defined with ``joints`` and ``command_interfaces`` parameters as follows: ``<joint>/<command_interface>``.
Supported state interfaces are ``position``, ``velocity``, and ``acceleration`` as defined in the `hardware_interface/hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_.
