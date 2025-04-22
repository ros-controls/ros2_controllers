:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/effort_controllers/doc/userdoc.rst

.. _effort_controllers_userdoc:

effort_controllers
==================

This is a collection of controllers that work using the "effort" joint command interface but may accept different joint-level commands at the controller level, e.g. controlling the effort on a certain joint to achieve a set position.

The package contains the following controllers:

effort_controllers/JointGroupEffortController
---------------------------------------------

This is specialization of the :ref:`forward_command_controller <forward_command_controller_userdoc>` that works using the "effort" joint interface.


ROS 2 interface of the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Topics
,,,,,,

~/commands (input topic) [std_msgs::msg::Float64MultiArray]
  Joints' effort commands


Parameters
,,,,,,,,,,
This controller overrides the interface parameter from :ref:`forward_command_controller <forward_command_controller_userdoc>`, and the
``joints`` parameter is the only one that is required.

An example parameter file is given here

.. code-block:: yaml

  controller_manager:
    ros__parameters:
      update_rate: 100  # Hz

      effort_controller:
        type: effort_controllers/JointGroupEffortController

  effort_controller:
    ros__parameters:
      joints:
        - slider_to_cart

Model-based controllers
--------------------------

Model-based controllers' control laws often require computing the manipulator's dynamics terms, e.g. inertia, coriolis, friction, and gravity contributions.
Notable examples are `gravity-compensation PD controllers or inverse dynamics controllers <https://doi.org/10.1007/978-1-84628-642-1>`_.

Control laws
^^^^^^^^^^^^

For instance, the PD Control with Gravity Compensation law is the following

.. math::

  \boldsymbol\tau = \boldsymbol K_p \tilde{\boldsymbol q} - \boldsymbol K_d \dot{\boldsymbol q} + \boldsymbol g(\boldsymbol q)

and can be implemented in C++ as

.. code-block:: cpp

  torque_output = K_p * position_error - K_d * velocity + g;

Similarly, the inverse dynamics control law is the following

.. math::

  \boldsymbol\tau = \boldsymbol B(\boldsymbol q) \left( \boldsymbol K_p \tilde{\boldsymbol q} + \boldsymbol K_d \dot{\tilde{\boldsymbol q}} + \ddot{\boldsymbol q}_d \right) + \boldsymbol C(\boldsymbol q, \dot{\boldsymbol q}) \dot{\boldsymbol q} + \boldsymbol f(\dot{\boldsymbol q}) + \boldsymbol g(\boldsymbol q)

and can be implemented in C++ as

.. code-block:: cpp

  torque_output = B * (K_p * position_error + K_d * velocity_error + desired_acceleration) + c + f + g;

The role of the inverse dynamics solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To facilitate the implementation of these controllers, the robot-agnostic `inverse dynamics solver <https://index.ros.org/r/inverse_dynamics_solver/github-unisa-acg-inverse-dynamics-solver/>`_ plugin can retrieve the dynamics terms with

.. code-block:: cpp

  Eigen::MatrixXd B = inverse_dynamics_solver_->getInertiaMatrix(position);
  Eigen::VectorXd c = inverse_dynamics_solver_->getCoriolisVector(position, velocity);
  Eigen::VectorXd f = inverse_dynamics_solver_->getFrictionVector(velocity);
  Eigen::VectorXd g = inverse_dynamics_solver_->getGravityVector(position);

where ``position``, ``velocity``, ``desired_acceleration``, ``position_error`` and ``velocity_error`` are given by the current robot state and reference, ``K_p`` and ``K_d`` are control gains, and ``torque_output`` shall be written on the command interfaces.

For more information about the solver, please have a look at `this example <https://github.com/unisa-acg/inverse-dynamics-solver/tree/humble/kdl_inverse_dynamics_solver#configuration>`_ with KDL for simulated robots.
