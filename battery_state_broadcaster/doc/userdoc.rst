:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/doc/userdoc.rst

.. _battery_state_broadcaster_userdoc:

Battery State Broadcaster
--------------------------------
The *Battery State Broadcaster* publishes battery status information as ``sensor_msgs/msg/BatteryState`` messages.

It reads battery-related state interfaces from one or more batteries and exposes them in a standard ROS 2 message format. This allows easy integration with monitoring tools, logging systems, and higher-level decision-making nodes.

Interfaces
^^^^^^^^^^^

The broadcaster can read the following state interfaces from each configured battery:

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

- ``~/raw_battery_states`` (``control_msgs/msg/BatteryStateArray``)
  Publishes **per-battery state messages**, containing the raw values for each configured battery.

- ``~/battery_state`` (``sensor_msgs/msg/BatteryState``)
  Publishes a **single aggregated battery message** representing the combined status across all batteries.

+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| Field                       | ``battery_state``                                                       | ``raw_battery_states``                                                                                                                          |
+=============================+=========================================================================+=================================================================================================================================================+
| ``header.frame_id``         | Empty                                                                   | Battery name                                                                                                                                    |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``voltage``                 | Mean across all batteries                                               | From battery's ``battery_voltage`` interface if enabled, otherwise nan                                                                          |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``temperature``             | Mean across batteries reporting temperature                             | From battery's ``battery_temperature`` interface if enabled, otherwise nan.                                                                     |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``current``                 | Mean across batteries reporting current                                 | From battery's ``battery_current`` interface if enabled, otherwise nan.                                                                         |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``charge``                  | Sum across all batteries                                                | From battery's ``battery_charge`` interface if enabled, otherwise nan.                                                                          |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``capacity``                | Sum across all batteries                                                | From battery's ``capacity`` parameter if provided, otherwise nan.                                                                               |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``design_capacity``         | Sum across all batteries                                                | From battery's ``design_capacity`` parameter if provided, otherwise nan.                                                                        |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``percentage``              | Mean across batteries reporting/calculating percentage                  | From battery's ``battery_percentage`` interface if enabled, otherwise calculated from battery's ``min_voltage`` and ``max_voltage`` parameters. |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_status``     | Highest reported enum value                                             | From battery's ``battery_power_supply_status`` interface if enabled, otherwise 0 (unknown).                                                     |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_health``     | Highest reported enum value                                             | From battery's ``battery_power_supply_health`` interface if enabled, otherwise 0 (unknown).                                                     |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``power_supply_technology`` | Reported as-is if same across all batteries, otherwise set to *Unknown* | From battery's ``power_supply_technology`` parameter if provided, otherwise 0 (unknown).                                                        |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``present``                 | True                                                                    | From battery's ``battery_present`` interface if enabled, otherwise true if battery's voltage values is valid.                                   |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``cell_voltage``            | Empty                                                                   | Empty                                                                                                                                           |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``cell_temperature``        | Empty                                                                   | Empty                                                                                                                                           |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``location``                | All battery locations appended                                          | From battery's ``location`` parameter if provided, otherwise empty.                                                                             |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+
| ``serial_number``           | All battery serial numbers appended                                     | From battery's ``serial_number`` parameter if provided, otherwise empty.                                                                        |
+-----------------------------+-------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------------------------------+


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

Migration for ``ipa320/ros_battery_monitoring`` users
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you were previously using the ``battery_state_broadcaster`` from the ``ipa320/ros_battery_monitoring package``, you can switch directly to this package. The configuration style using ``sensor_name`` is still supported for backward compatibility, but it may be removed in a future release.

To adapt your setup to the new ``battery_state_broadcaster`` configuration:

1. Update your hardware interface name from ``voltage`` → ``battery_voltage``.

2. Convert your controller parameters from

  .. code-block:: yaml

    battery_state_broadcaster:
      ros__parameters:
        sensor_name: "battery_state"
        design_capacity: 100.0
        # https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg
        power_supply_technology: 2

  to:

  .. code-block:: yaml

    battery_state_broadcaster:
      ros__parameters:
        batteries: ["battery_state"]
        battery_state:
          design_capacity: 100.0
          power_supply_technology: 2

**Notes**:

- Parameters must provide **either** sensor_name **or** batteries.
- If both are empty → the broadcaster will fail to configure.
- If both are set → the broadcaster will throw an error.
