:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/userdoc.rst

.. _motion_primitives_controllers_userdoc:


.. include:: README.md
   :parser: myst_parser.sphinx_

Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/src/motion_primitives_forward_controller_parameter.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: src/motion_primitives_forward_controller_parameter.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/motion_primitives_controllers/test/motion_primitives_forward_controller_params.yaml>`_:

.. literalinclude:: test/motion_primitives_forward_controller_params.yaml
   :language: yaml
