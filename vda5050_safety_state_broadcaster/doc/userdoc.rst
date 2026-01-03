:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/vda5050_safety_state_broadcaster/doc/userdoc.rst

.. _vda5050_safety_state_broadcaster_userdoc:

VDA5050 Safety State Broadcaster
--------------------------------
The *VDA5050 Safety State Broadcaster* publishes safety state information as ``control_msgs/msg/VDA5050SafetyState`` messages, as defined by the VDA5050 standard.

It reads safety-related state interfaces from a ros2_control system and exposes them in a standard ROS 2 message format. This enables easy integration with VDA5050-compliant systems, safety monitoring, and higher-level fleet management.

Interfaces
^^^^^^^^^^

The broadcaster can read the following state interfaces, configured via parameters:

- ``fieldViolation_interfaces`` (string_array): interface names used for field violation events by setting value to 1.0 or true.
- ``eStop_manual_interfaces`` (string_array): interface names used for manual eStop events by setting value to 1.0 or true.
- ``eStop_remote_interfaces`` (string_array): interface names used for remote eStop events by setting value to 1.0 or true.
- ``eStop_autoack_interfaces`` (string_array): interface names used for autoAck eStop events by setting value to 1.0 or true.

NOTE: The broadcaster supports both double and bool hardware interfaces.

Published Topics
^^^^^^^^^^^^^^^^

The broadcaster publishes the following topic:

- ``~/vda5050_safety_state`` (``control_msgs/msg/VDA5050SafetyState``)
  Publishes the **combined safety state** of the fieldViolation and eStop interfaces, with priority: eStop_manual > eStop_remote > eStop_autoack for eStop.

Message Fields
^^^^^^^^^^^^^^

The published ``VDA5050SafetyState`` message contains:

+--------------------+-------------------------------------------------------------------------------------------------------+
| Field              | Description                                                                                           |
+====================+=======================================================================================================+
| ``field_violation``| True if any field violation interface is active                                                       |
+--------------------+-------------------------------------------------------------------------------------------------------+
| ``e_stop``         | E-stop state, one of:                                                                                 |
|                    |                                                                                                       |
|                    | - ``none``: No E-stop active                                                                          |
|                    | - ``manual``: Any Manual E-stop Interface triggered                                                   |
|                    | - ``remote``: Any Remote E-stop Interface triggered and manual is not active                          |
|                    | - ``autoAck``: Any Auto-acknowledged E-stop Interface triggered and manual and remote are not active  |
+--------------------+-------------------------------------------------------------------------------------------------------+

The E-stop state is determined by the first active interface in the following priority:
``manual > remote > autoAck > none``.

Parameters
^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to manage parameters.
The parameter `definition file <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/vda5050_safety_state_broadcaster/src/vda5050_safety_state_broadcaster.yaml>`_ contains the full list and descriptions.

List of parameters
==================
.. generate_parameter_library_details:: ../src/vda5050_safety_state_broadcaster.yaml

Example Parameter File
======================

An example parameter file for this controller is available in the `test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/vda5050_safety_state_broadcaster/test/vda5050_safety_state_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/vda5050_safety_state_broadcaster_params.yaml
   :language: yaml
