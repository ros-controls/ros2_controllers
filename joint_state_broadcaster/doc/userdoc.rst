.. _joint_state_broadcaster_userdoc:

joint_state_broadcaster
----------------------

The broadcaster reads all state interfaces and reports them on ``/joint_states`` and ``/dynamic_joint_states``.

Commands
^^^^^^^^

Broadcasters are not real controllers, and therefore take no commands.

Hardware interface type
^^^^^^^^^^^^^^^^^^^^^^^

All available *joint state interfaces* are used by this broadcaster.

Parameters
^^^^^^^^^^

``joints``

  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with ``interfaces`` parameters.
  Joint state broadcaster asks for access to all defined interfaces on all defined joints.


``interfaces``

  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with ``joints`` parameters.


``extra_joints``

  Optional parameter (string array) with names of extra joints to be added to ``joint_states`` and ``dynamic_joint_states`` with state set to 0.


``map_interface_to_joint_state``

  Optional parameter (map) providing mapping between custom inteface names to standard fields in ``joints_states` message.
  Usecases:

    1. Hydraulics robots where feedback and commanded values have offset.
       Typically one would map both values in separete interfaces inthe framework.
       To visualize those data mutiple JSB's and robot_state_publishers would be used to visualize both values in RViz.

    1. A robot provides different measuring tequniques for its joint values which results in slightly differnt values.
       Typically one would use separate interface for providing those values in the framework.
       Using multiple JSB's we could publish and show both in RViz.

  Format (each line is optional):

  .. code-block:: yaml

     map_interface_to_joint_state
       position: <custom_interfce>
       velocity: <custom_interface>
       effort: <custom_interface>
