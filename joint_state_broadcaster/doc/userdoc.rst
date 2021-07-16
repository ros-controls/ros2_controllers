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

extra_joints (optional; string array; default: empty)
  Names of extra joints to be added to ``/joint_states`` and ``/dynamic_joint_states`` with state set to 0.
