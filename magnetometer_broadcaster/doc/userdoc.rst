:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/magnetometer_broadcaster/doc/userdoc.rst

.. _magnetometer_broadcaster_userdoc:

Magnetometer Broadcaster
--------------------------------
Broadcaster for magnetometer messages (type: ``sensor_msgs/msg/MagneticField``), using the ``semantic_components::MagneticFieldSensor``.

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/magnetometer_broadcaster/src/magnetometer_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

List of parameters
=========================
.. generate_parameter_library_details:: ../src/magnetometer_broadcaster_parameters.yaml


An example parameter file
=========================

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/magnetometer_broadcaster/test/magnetometer_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/magnetometer_broadcaster_params.yaml
   :language: yaml
