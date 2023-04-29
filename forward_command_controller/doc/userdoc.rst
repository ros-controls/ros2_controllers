.. _forward_command_controller_userdoc:

forward_command_controller
==========================

<<<<<<< HEAD
This is a collection of controllers that work using the "effort" joint command interface but may accept different joint-level commands at the controller level, e.g. controlling the effort on a certain joint to achieve a set position.
=======
This is a base class implementing a feedforward controller. Specific implementations can be found in:

* :ref:`position_controllers_userdoc`
* :ref:`velocity_controllers_userdoc`
* :ref:`effort_controllers_userdoc`
>>>>>>> 8da5135 (Fix docs format (#589))

Hardware interface type
-----------------------

These controllers work with joints using the "effort" command interface.
