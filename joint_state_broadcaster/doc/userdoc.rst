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

The parameter ``extra_joints`` is optional and is an array of strings consisting of the names of the extra joints that will be added to ``/joint_states`` and ``/dynamic_joint_states`` with state set to 0.
