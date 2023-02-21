.. _tricycle_steering_controller_userdoc:

tricycle_steering_controller
=============================

This controller implements the kinematics with two axes and three wheels, where two wheels on an axis are fixed (traction/drive), and the wheel on the other axis is steerable.

The controller expects to have two commanding joints for traction, one for each fixed wheel and one commanding joint for steering.

For more details on controller's execution and interfaces check the :ref:`Steering Controller Library <steering_controllers_library>`.


Parameters
,,,,,,,,,,,

For list of parameters and their meaning YAML file in the ``src`` folder of the controller's package.

For an exemplary parameterization see the ``test`` folder of the controller's package.
