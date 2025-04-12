:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/bicycle_steering_controller/doc/userdoc.rst

.. _bicycle_steering_controller_userdoc:

bicycle_steering_controller
=============================

This controller implements the kinematics with two axes and two wheels, where the wheel on one axis is fixed (traction/drive), and the wheel on the other axis is steerable.

The controller expects to have one commanding joint for traction, and one commanding joint for steering.
If your Ackermann steering vehicle uses differentials on axes, then you should probably use this controller since you can command only one traction velocity and steering angle for virtual wheels in the middle of the axes.

For more details on controller's execution and interfaces check the :ref:`Steering Controller Library <steering_controllers_library_userdoc>`.


Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

Additionally to the parameters of the :ref:`Steering Controller Library <doc/ros2_controllers/steering_controllers_library/doc/userdoc:parameters>`, this controller adds the following parameters:

.. generate_parameter_library_details:: ../src/bicycle_steering_controller.yaml
