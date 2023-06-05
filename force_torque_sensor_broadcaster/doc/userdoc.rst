:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster/doc/userdoc.rst

.. _force_torque_sensor_broadcaster_userdoc:

Force Torque Sensor Broadcaster
--------------------------------
Broadcaster of messages from force/torque state interfaces of a robot or sensor.
The published message type is ``geometry_msgs/msg/WrenchStamped``.

The controller is a wrapper around ``ForceTorqueSensor`` semantic component (see ``controller_interface`` package).


Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

The interfaces can be defined in two ways, using ``sensor_name`` or ``interface_names`` parameter:
Those two parameters cannot be defined at the same time.

interface_names.[force|torque].[x|y|z] (optional)
  Defines custom, per axis interface names.
  This is used if different prefixes, i.e., sensor name, or non-standard interface names are used.
  It is sufficient that only one ``interface_name`` is defined.
  This enables broadcaster use for force sensing cells with less then six measuring axes.
  Example definition is:

  .. code-block:: yaml

     interface_names:
       force:
         x: example_name/example_interface

Full list of parameters:

.. generate_parameter_library_details:: ../src/force_torque_sensor_broadcaster_parameters.yaml
