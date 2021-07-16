.. _joint_state_broadcaster_userdoc:

joint_state_broadcaster
=======================

The broadcaster reads all state interfaces and reports them on ``/joint_states`` and ``/dynamic_joint_states``.

Commands
--------

Broadcasters are not real controllers, and therefore take no commands.

Hardware interface type
-----------------------

All available *joint state interfaces* are used by this broadcaster.

Parameters
----------

``use_local_topics``

  Optional parameter (boolean; default: ``False``) defining if ``joint_states`` and ``dynamic_joint_states`` messages should be published into local namespace, e.g., ``/my_state_broadcaster/joint_states``.


``joints``
  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with ``interfaces`` parameters.
  Joint state broadcaster asks for access to all defined interfaces on all defined joints.


``interfaces``
  Optional parameter (string array) to support broadcasting of only specific joints and interfaces.
  It has to be used in combination with ``joints`` parameters.


``extra_joints``

  Optional parameter (string array) with names of extra joints to be added to ``joint_states`` and ``dynamic_joint_states`` with state set to 0.
