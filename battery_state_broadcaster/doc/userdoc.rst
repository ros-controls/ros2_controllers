:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/doc/userdoc.rst

.. _battery_state_broadcaster_userdoc:

Battery State Broadcaster
--------------------------------
The *Battery State Broadcaster* publishes battery status information as ``sensor_msgs/msg/BatteryState`` messages.

It reads battery-related state interfaces from one or more joints and exposes them in a standard ROS 2 message format. This allows easy integration with monitoring tools, logging systems, and higher-level decision-making nodes.

Interfaces
^^^^^^^^^^^

The broadcaster can read the following state interfaces from each configured joint:

- ``battery_voltage`` *(mandatory)* (double)
- ``battery_temperature`` *(optional)* (double)
- ``battery_current`` *(optional)* (double)
- ``battery_charge`` *(optional)* (double)
- ``battery_percentage`` *(optional)* (double)
- ``battery_power_supply_status`` *(optional)* (double)
- ``battery_power_supply_health`` *(optional)* (double)
- ``battery_present`` *(optional)* (bool)

Published Topics
^^^^^^^^^^^^^^^^^^

The broadcaster publishes two topics:

- ``~/raw_battery_states`` (``control_msgs/msg/BatteryStates``)
  Publishes **per-joint battery state messages**, containing the raw values for each configured joint.

- ``~/battery_state`` (``sensor_msgs/msg/BatteryState``)
  Publishes a **single aggregated battery message** representing the combined status across all joints.

+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| Field                       | ``battery_state``                                                    | ``raw_battery_states``                                                                                                                      |
+=============================+======================================================================+=============================================================================================================================================+
| ``header.frame_id``         | Empty                                                                | Joint name                                                                                                                                  |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``voltage``                 | Mean across all joints                                               | From joint's ``battery_voltage`` interface if enabled, otherwise nan                                                                        |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``temperature``             | Mean across joints reporting temperature                             | From joint's ``battery_temperature`` interface if enabled, otherwise nan.                                                                   |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``current``                 | Mean across joints reporting current                                 | From joint's ``battery_current`` interface if enabled, otherwise nan.                                                                       |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``charge``                  | Sum across all joints                                                | From joint's ``battery_charge`` interface if enabled, otherwise nan.                                                                        |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``capacity``                | Sum across all joints                                                | From joint's ``capacity`` parameter if provided, otherwise nan.                                                                             |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``design_capacity``         | Sum across all joints                                                | From joint's ``design_capacity`` parameter if provided, otherwise nan.                                                                      |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``percentage``              | Mean across joints reporting/calculating percentage                  | From joint's ``battery_percentage`` interface if enabled, otherwise calculated from joint's ``min_voltage`` and ``max_voltage`` parameters. |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_status``     | Highest reported enum value                                          | From joint's ``battery_power_supply_status`` interface if enabled, otherwise 0 (unknown).                                                   |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_health``     | Highest reported enum value                                          | From joint's ``battery_power_supply_health`` interface if enabled, otherwise 0 (unknown).                                                   |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_technology`` | Reported as-is if same across all joints, otherwise set to *Unknown* | From joint's ``power_supply_technology`` parameter if provided, otherwise 0 (unknown).                                                      |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``present``                 | True                                                                 | From joint's ``battery_present`` interface if enabled, otherwise true if joint's voltage values is valid.                                   |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``cell_voltage``            | Empty                                                                | Empty                                                                                                                                       |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``cell_temperature``        | Empty                                                                | Empty                                                                                                                                       |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``location``                | All joint locations appended                                         | From joint's ``location`` parameter if provided, otherwise empty.                                                                           |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+
| ``serial_number``           | All joint serial numbers appended                                    | From joint's ``serial_number`` parameter if provided, otherwise empty.                                                                      |
+-----------------------------+----------------------------------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------+


Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to manage parameters.
The parameter `definition file <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/src/battery_state_broadcaster_parameters.yaml>`_ contains the full list and descriptions.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/battery_state_broadcaster_parameters.yaml

Example Parameter File
=========================

An example parameter file for this controller is available in the `test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/test/battery_state_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/battery_state_broadcaster_params.yaml
   :language: yaml
