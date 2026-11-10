:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/admittance_controller/doc/userdoc.rst

.. _admittance_controller_userdoc:

Admittance Controller
======================

Admittance controller enables you do zero-force control from a force measured on your TCP.
The controller implements ``ChainedControllerInterface``, so it is possible to add another controllers in front of it, e.g., ``JointTrajectoryController``.

The controller requires an external kinematics plugin to function. The `kinematics_interface <https://github.com/ros-controls/kinematics_interface>`_ repository provides an interface and an implementation that the admittance controller can use.


ROS 2 interface of the controller
---------------------------------

Parameters
^^^^^^^^^^^

The admittance controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/admittance_controller/src/admittance_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/admittance_controller_parameters.yaml

Wrench filtering
""""""""""""""""
The measured wrench is processed by a filter chain defined in the ``sensor_filter_chain`` parameter namespace, e.g., with ``control_filters/GravityCompensationWrench`` and ``control_filters/LowPassFilterWrench`` from `control_toolbox <https://github.com/ros-controls/control_toolbox>`_. Without filters, the measured wrench is used as is.
The output of the chain must be in ``ft_sensor.frame.id``. If the sensor measures in another frame (``ft_sensor.measurement_frame.id``), a filter of the chain has to transform the wrench accordingly.

.. note::
  ``control_filters/GravityCompensationWrench`` looks up its frames via ``tf2`` in every update, i.e., inside the real-time loop. These lookups lock the tf buffer, which is shared with the tf listener thread, so they are not real-time safe and can cause jitter on a busy tf tree. The tf data also lags behind the joint states used by the kinematics plugin.

If an update fails (e.g., a kinematics call fails or a filter cannot look up its frames), the controller holds its last commanded positions, commands zero velocities and accelerations, and logs an error.

The parameters ``gravity_compensation.*``, ``fixed_world_frame.frame.id`` and ``ft_sensor.filter_coefficient`` are deprecated. If ``gravity_compensation.frame.id`` is set, the old built-in gravity compensation and exponential filter are used instead of the filter chain.

An example parameter file for this controller can be found in `the test folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/admittance_controller/test/test_params.yaml>`_:

.. literalinclude:: ../test/test_params.yaml
   :language: yaml

Topics
^^^^^^^

~/joint_references (input topic) [trajectory_msgs::msg::JointTrajectoryPoint]
  Target joint commands when controller is not in chained mode.

~/wrench_reference (input topic) [geometry_msgs::msg::WrenchStamped]
  Target wrench offset (WrenchStamped has to be in ``ft_sensor.frame.id``). It is added to the measured wrench after ``sensor_filter_chain``, or before the deprecated built-in gravity compensation and exponential filter.

~/status (output topic) [control_msgs::msg::AdmittanceControllerState]
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
Supported state interfaces are ``position``, ``velocity``, and ``acceleration`` as defined in the `hardware_interface/hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_.
If some interface is not provided, the last commanded interface will be used for calculation.

For handling TCP wrenches `*Force Torque Sensor* semantic component  (from package *controller_interface*) <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/semantic_components/force_torque_sensor.hpp>`_ is used.
The interfaces have prefix ``ft_sensor.name``, building the interfaces: ``<sensor_name>/[force.x|force.y|force.z|torque.x|torque.y|torque.z]``.


Commands
^^^^^^^^^
The command interfaces are defined with ``joints`` and ``command_interfaces`` parameters as follows: ``<joint>/<command_interface>``.
Supported command interfaces are ``position``, ``velocity``, and ``acceleration`` as defined in the `hardware_interface/hardware_interface_type_values.hpp <https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`_.
