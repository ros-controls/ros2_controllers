:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gravity_compensation_pd_controller/doc/userdoc.rst

.. _gravity_compensation_pd_controller_userdoc:

Gravity Compensation PD Controller
==================================

This controller provides a PD controller with gravity compensation that converts joint position references to joint effort commands for a robotic manipulator.
It is a chainable controller; therefore, it requires another controller to provide joint position references via chaining.

The controller uses the robot's dynamic model to compute the control action.
In particular, it relies on the `InverseDynamicsSolver <https://index.ros.org/p/inverse_dynamics_solver/>`_ interface to estimate the gravity torque vector.

Control Law
-----------

The implemented control law follows the scheme shown in the figure, based on the formulation presented in *B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, "Robotics: Modelling, Planning and Control"*.

.. image:: ../doc/media/gravity_compensation_pd_controller_scheme.png
   :alt: Gravity Compensation PD Controller Scheme


Description of controller's interfaces
--------------------------------------

References (from a preceding controller)
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
- ``<joint_name>/position`` [double]

Commands
,,,,,,,,
- ``<joint_name>/effort`` [double]

States
,,,,,,
- ``<joint_name>/position`` [double]
- ``<joint_name>/velocity`` [double]


Behavior in Edge Cases
----------------------

The controller is designed to handle the following edge cases:

- **Computed torque exceeding joint limits**. If the computed torque exceeds the joint limits, the controller will saturate the output to remain within the range specified in the robot description.

- **Invalid joint position reference**. If the joint position reference is ``NaN``, the controller will fall back to the last valid reference to compute the control action. Initially, the first valid reference is set to the robot's starting joint positions.

- **Invalid joint position state**. If the controller detects a ``NaN`` value in the joint position state reported by the robot, it will return an error to the controller manager. In this case, the controller cannot compute a valid control action and will output ``NaN`` values. It is assumed that the hardware interface is capable of handling ``NaN`` values appropriately.


Parameters
----------

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gravity_compensation_pd_controller/src/gravity_compensation_pd_controller_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/gravity_compensation_pd_controller_parameters.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/gravity_compensation_pd_controller/test/config/test_gravity_compensation_pd_controller.yaml>`_:

.. literalinclude:: ../test/config/test_gravity_compensation_pd_controller.yaml
   :language: yaml
