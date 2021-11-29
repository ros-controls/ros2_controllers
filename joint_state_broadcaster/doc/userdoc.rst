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
