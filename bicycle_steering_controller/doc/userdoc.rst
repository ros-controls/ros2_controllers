.. _bicycle_steering_controller_userdoc:

bicycle_steering_controller
=============================

This controller implements the kinematics with two axes and two wheels, where the wheel on one axis is fixed (traction/drive), and the wheel on the other axis is steerable.

The controller expects to have one commanding joint for traction, and one commanding joint for steering.
If your Ackermann steering vehicle uses differentials on axes, then you should probably use this controller since you can command only one traction velocity and steering angle for virtual wheels in the middle of the axes.

For more details on controller's execution and interfaces check the :ref:`Steering Controller Library <steering_controllers_library>`.


Parameters
,,,,,,,,,,,

For list of parameters and their meaning YAML file in the ``src`` folder of the controller's package.

For an exemplary parameterization see the ``test`` folder of the controller's package.
