:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/ackermann_steering_controller/doc/userdoc.rst

.. _ackermann_steering_controller_userdoc:

ackermann_steering_controller
=============================

This controller implements the kinematics with two axes and four wheels, where the wheels on one axis are fixed (traction/drive) wheels, and the wheels on the other axis are steerable.

The controller expects to have two commanding joints for traction, one for each fixed wheel and two commanding joints for steering, one for each wheel.

For more details on controller's execution and interfaces check the :ref:`Steering Controller Library <steering_controllers_library_userdoc>`.


Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

Additionally to the parameters of the :ref:`Steering Controller Library <doc/ros2_controllers/steering_controllers_library/doc/userdoc:parameters>`, this controller adds the following parameters:

.. generate_parameter_library_details:: ../src/ackermann_steering_controller.yaml
