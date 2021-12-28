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

Parameters
----------

``use_local_topics``
  Optional parameter (boolean; default: ``False``) defining if ``joint_states`` and ``dynamic_joint_states`` messages should be published into local namespace, e.g., ``/my_state_broadcaster/joint_states``.


``joints``
  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with the ``interfaces`` parameter.
  Joint state broadcaster asks for access to all defined interfaces on all defined joints.


``interfaces``
  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with the ``joints`` parameter.


``extra_joints``
  Optional parameter (string array) with names of extra joints to be added to ``joint_states`` and ``dynamic_joint_states`` with state set to 0.


``map_interface_to_joint_state``
  Optional parameter (map) providing mapping between custom interface names to standard fields in ``joint_states`` message.
  Usecases:

    1. Hydraulics robots where feedback and commanded values often have an offset and reliance on open-loop control is common.
       Typically one would map both values in separate interfaces in the framework.
       To visualize those data multiple joint_state_broadcaster instances and robot_state_publishers would be used to visualize both values in RViz.

    1. A robot provides multiple measuring techniques for its joint values which results in slightly different values.
       Typically one would use separate interface for providing those values in the framework.
       Using multiple joint_state_broadcaster instances we could publish and show both in RViz.

  Format (each line is optional):

  .. code-block:: yaml

      map_interface_to_joint_state:
        position: <custom_interface>
        velocity: <custom_interface>
        effort: <custom_interface>


  Examples:

  .. code-block:: yaml

      map_interface_to_joint_state:
        position: kf_estimated_position


  .. code-block:: yaml

      map_interface_to_joint_state:
        velocity: derived_velocity
        effort: derived_effort


  .. code-block:: yaml

      map_interface_to_joint_state:
        effort: torque_sensor


  .. code-block:: yaml

      map_interface_to_joint_state:
        effort: current_sensor
