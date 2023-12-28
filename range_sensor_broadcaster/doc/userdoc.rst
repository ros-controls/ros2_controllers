:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/range_sensor_broadcaster/doc/userdoc.rst

.. _range_sensor_broadcaster_userdoc:

Range Sensor Broadcaster
--------------------------------
Broadcaster of messages from Range sensor.
The published message type is ``sensor_msgs/msg/Range``.

The controller is a wrapper around ``RangeSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
The Range Sensor Broadcaster uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/range_sensor_broadcaster_parameters.yaml


An example parameter file
=========================

.. generate_parameter_library_default::
  ../src/range_sensor_broadcaster_parameters.yaml
