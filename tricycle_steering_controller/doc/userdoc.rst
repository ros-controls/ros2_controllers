:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/tricycle_steering_controller/doc/userdoc.rst

.. _tricycle_steering_controller_userdoc:

tricycle_steering_controller
=============================

This controller implements the kinematics with two axes and three wheels.

Two possible configurations are supported:
1. Two wheels on an axis are fixed (traction/drive), and the wheel on the other axis is steerable. The controller expects to have two commanding joints for traction, one for each fixed wheel and one commanding joint for steering.
2. A single traction wheel, which is also steerable: The controller expects a single command joint for traction and one commanding joint for steering.


For more details on controller's execution and interfaces check the :ref:`Steering Controller Library <steering_controllers_library_userdoc>`.


Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

Additionally to the parameters of the :ref:`Steering Controller Library <doc/ros2_controllers/steering_controllers_library/doc/userdoc:parameters>`, this controller adds the following parameters:

.. generate_parameter_library_details:: ../src/tricycle_steering_controller.yaml
