:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster/doc/userdoc.rst

.. _force_torque_sensor_broadcaster_userdoc:

Force Torque Sensor Broadcaster
--------------------------------
Broadcaster of messages from force/torque state interfaces of a robot or sensor.
The published message type is ``geometry_msgs/msg/WrenchStamped``.

The broadcaster also supports filtering the force/torque readings using a filter chain, enabling the sequential application of multiple filters. In this case, an additional topic with the
``_filtered`` suffix will be published containing the final result. For more details on filters, refer to the `filters package repository <https://github.com/ros/filters>`_.
See the parameter section for instructions on configuring a filter chain with an arbitrary number of filters.

The controller is a wrapper around ``ForceTorqueSensor`` semantic component (see ``controller_interface`` package).

Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster/src/force_torque_sensor_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

The interfaces can be defined in two ways, using the ``sensor_name`` or the ``interface_names`` parameter:
Those two parameters cannot be defined at the same time.

The filter chain is configured as a sequential list of filters under the ``sensor_filter_chain`` parameter, where each filter is identified by the key ``filterN``, with ``N`` representing its position in the chain (e.g., ``filter1``, ``filter2``, etc.).
Each filter entry must specify the ``name`` and ``type`` of the plugin, along with any additional parameters required by that specific filter plugin. The chain will use the `pluginlib <https://github.com/ros/pluginlib>`_ library to load each filter at runtime,
passing the specified parameters.

The chain processes the data sequentially, passing the output of one filter as the input to the next.

Full list of parameters:

.. generate_parameter_library_details:: ../src/force_torque_sensor_broadcaster_parameters.yaml
  parameters_context.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster/test/force_torque_sensor_broadcaster_params.yaml>`_:

.. literalinclude:: ../test/force_torque_sensor_broadcaster_params.yaml
   :language: yaml

Wrench Transformer Node
-----------------------
The package provides a standalone ROS 2 node ``wrench_transformer_node`` that transforms wrench messages published by the ``ForceTorqueSensorBroadcaster`` controller to different target frames using TF2. This is useful when applications need force/torque data in coordinate frames other than the sensor frame.

The node subscribes to wrench messages from the broadcaster (either raw or filtered) and publishes transformed versions to separate topics for each target frame.

Usage
^^^^^
The wrench transformer node can be launched as a standalone executable:

.. code-block:: bash

   ros2 run force_torque_sensor_broadcaster wrench_transformer_node

Wrench Transformer Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The wrench transformer uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file for the wrench transformer <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster/src/wrench_transformer_parameters.yaml>`_ contains descriptions for all the parameters.

Full list of parameters:

.. generate_parameter_library_details:: ../src/wrench_transformer_parameters.yaml

Topics
^^^^^^
The node subscribes to:

- ``~/wrench`` (raw wrench messages). To subscribe to filtered wrench messages, use topic remapping: ``ros2 run ... --ros-args -r ~/wrench:=<namespace>/wrench_filtered``

The node publishes:

- ``<namespace>/<target_frame>/wrench`` for each target frame specified in ``target_frames``

  - If the node is in the root namespace (``/``), the namespace defaults to the node name (e.g., ``/fts_wrench_transformer/<target_frame>/wrench``)
  - If the input topic is remapped to a filtered topic (contains "filtered" in the name), the output topics automatically append ``_filtered`` suffix (e.g., ``<namespace>/<target_frame>/wrench_filtered``)
  - This allows users to distinguish between transformed raw wrench data and transformed filtered wrench data
