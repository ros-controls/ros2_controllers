.. _force_torque_sensor_broadcaster_userdoc:

Force Torque Sensor Broadcaster
--------------------------------
Broadcaster of messages from force/torque state interfaces of a robot or sensor.
The published message type is ``geometry_msgs/msg/WrenchStamped``.

The controller is a wrapper around ``ForceTorqueSensor`` semantic component (see ``controller_interface`` package).


Parameters
^^^^^^^^^^^
The interfaces can be defined in two ways, using ``sensor_name`` or ``interface_names`` parameter.
Those two parameters can not be defined at the same time

frame_id (mandatory)
  Frame in which the output message will be published.

sensor_name (optional)
  Defines sensor name used as prefix for its interfaces.
  If used standard interface names for a 6D FTS will be used: <sensor_name>/force.x, ..., <sensor_name>/torque.z.

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
