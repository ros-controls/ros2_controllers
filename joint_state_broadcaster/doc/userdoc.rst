:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_state_broadcaster/doc/userdoc.rst

.. _joint_state_broadcaster_userdoc:

joint_state_broadcaster
=======================

The broadcaster reads all state interfaces and reports them on ``/joint_states`` and ``/dynamic_joint_states``.

Commands
--------

Broadcasters are not real controllers, and therefore take no commands.

Hardware interface type
-----------------------

By default, all available *joint state interfaces* are used, unless configured otherwise.
In the latter case, resulting interfaces is defined by a "matrix" of interfaces defined by the cross-product of the ``joints`` and ``interfaces`` parameters.
If some requested interfaces are missing, the controller will print a warning about that, but work for other interfaces.
If none of the requested interface are not defined, the controller returns error on activation.

Published topics
----------------

* ``/joint_states`` (``sensor_msgs/msg/JointState``):

  Publishes *movement-related* interfaces only — ``position``, ``velocity``,
  and ``effort`` — for joints that provide them. If a joint does not expose a given
  movement interface, that field is omitted/left empty for that joint in the message.

* ``/dynamic_joint_states`` (``control_msgs/msg/DynamicJointState``):

  Publishes **all available state interfaces** for each joint. This includes the
  movement interfaces (position/velocity/effort) *and* any additional or custom
  interfaces provided by the hardware (e.g., temperature, voltage, torque sensor
  readings, calibration flags).

  The message maps ``joint_names`` to per-joint interface name lists and values.
  Example payload::

    joint_names: [joint1, joint2]
    interface_values:
      - interface_names: [position, velocity, effort]
        values: [1.5708, 0.0, 0.20]
      - interface_names: [position, temperature]
        values: [0.7854, 42.1]

  Use this topic if you need *every* reported interface, not just movement.

.. note::

   If ``use_local_topics`` is set to ``true``, both topics are published in the
   controller’s namespace (e.g., ``/my_state_broadcaster/joint_states`` and
   ``/my_state_broadcaster/dynamic_joint_states``). If ``false`` (default),
   they are published at the root (e.g., ``/joint_states``).


Parameters
----------
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_state_broadcaster/src/joint_state_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


List of parameters
,,,,,,,,,,,,,,,,,,

.. generate_parameter_library_details::
  ../src/joint_state_broadcaster_parameters.yaml
  joint_state_broadcaster_parameter_context.yml


An example parameter file
,,,,,,,,,,,,,,,,,,,,,,,,,

.. generate_parameter_library_default::
  ../src/joint_state_broadcaster_parameters.yaml
