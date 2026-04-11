:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/userdoc.rst

.. _motion_primitives_controllers_userdoc:

motion_primitive_controllers
==========================================

Package to control robots using motion primitives like LINEAR_JOINT (PTP/ MOVEJ), LINEAR_CARTESIAN (LIN/ MOVEL) and CIRCULAR_CARTESIAN (CIRC/ MOVEC)

.. warning::
   There is no guarantee that the motion defined by the motion primitive will actually be executed exactly as planned. In particular, for motions in Cartesian space such as LIN primitives, it is not necessarily ensured that the robot will execute the motion exactly in that way, since the inverse kinematics is not always unique.

.. include:: README.md
   :parser: myst_parser.sphinx_

Parameters
----------

These controllers use the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle their parameters.

Parameter of motion_primitives_forward_controller
.................................................

.. _definition_file_forward:

The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/src/motion_primitives_forward_controller_parameter.yaml>`__ contains descriptions for all the parameters used by this controller.

.. generate_parameter_library_details:: ../src/motion_primitives_forward_controller_parameter.yaml

.. _test_file_forward:

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/test/motion_primitives_forward_controller_params.yaml>`__:

.. literalinclude:: ../test/motion_primitives_forward_controller_params.yaml
   :language: yaml

Parameter of motion_primitives_from_trajectory_controller
...........................................................

.. _definition_file_trajectory:

The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/src/motion_primitives_from_trajectory_controller_parameter.yaml>`__ contains descriptions for all the parameters used by this controller.

.. generate_parameter_library_details:: ../src/motion_primitives_from_trajectory_controller_parameter.yaml

.. _test_file_trajectory:

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/test/motion_primitives_from_trajectory_controller_params.yaml>`__:

.. literalinclude:: ../test/motion_primitives_from_trajectory_controller_params.yaml
   :language: yaml
