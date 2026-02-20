:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/userdoc.rst

.. _motion_primitives_controllers_userdoc:


.. include:: README.md
   :parser: myst_parser.sphinx_


Parameters
,,,,,,,,,,,

These controllers use the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle their parameters.

motion_primitives_forward_controller
------------------------------------

.. _definition_file_forward:

The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/src/motion_primitives_forward_controller_parameter.yaml>`__ contains descriptions for all the parameters used by this controller.

.. generate_parameter_library_details:: src/motion_primitives_forward_controller_parameter.yaml

.. _test_file_forward:

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/test/motion_primitives_forward_controller_params.yaml>`__:

.. literalinclude:: test/motion_primitives_forward_controller_params.yaml
   :language: yaml


motion_primitives_from_trajectory_controller
------------------------------------

.. _definition_file_trajectory:

The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/src/motion_primitives_from_trajectory_controller_parameter.yaml>`__ contains descriptions for all the parameters used by this controller.

.. generate_parameter_library_details:: src/motion_primitives_from_trajectory_controller_parameter.yaml

.. _test_file_trajectory:

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/test/motion_primitives_from_trajectory_controller_params.yaml>`__:

.. literalinclude:: test/motion_primitives_from_trajectory_controller_params.yaml
   :language: yaml
