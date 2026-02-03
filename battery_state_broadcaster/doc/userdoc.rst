:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/battery_state_broadcaster/doc/userdoc.rst

.. _battery_state_broadcaster_userdoc:

Battery State Broadcaster
=========================
This broadcaster publishes `sensor_msgs/BatteryState <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg>`__ messages from appropriate state interfaces.


Required State Interfaces
---------------------------------
This broadcaster requires the robot to have a sensor component which contains the battery state interfaces:

.. code-block:: xml

  <ros2_control type="system">

      <!-- ... -->

      <sensor name="battery_state">
          <state_interface name="voltage" />
      </sensor>
  </ros2_control>

Parameters
---------------------------------
To use this broadcaster, declare it in the controller manager and set its parameters:

.. code-block:: yaml

  controller_manager:
    ros__parameters:
      battery_state_broadcaster:
        type: battery_state_broadcaster/BatteryStateBroadcaster

  battery_state_broadcaster:
    ros__parameters:
      sensor_name: "battery_state"
      design_capacity: 100.0
      # https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg
      power_supply_technology: 2

And spawn it in the launch file:

.. code-block:: python

  battery_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["battery_state_broadcaster"]

Topics
---------------------------------
The battery state is published on ``~/battery_state``.
Since it's a plugin within the controller manager, add a remapping of the form ``("~/battery_state", "/my_battery_state")`` to the *controller manager*, not the spawner, to change the topic name.
