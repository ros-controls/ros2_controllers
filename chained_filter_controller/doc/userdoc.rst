:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/chained_filter_controller/doc/userdoc.rst

.. _chained_filter_controller_userdoc:

Chained Filter Controller
--------------------------------
tbd.


Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/chained_filter_controller/src/chained_filter_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


Full list of parameters:

.. generate_parameter_library_details:: ../src/chained_filter_parameters.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/chained_filter_controller/test/config/test_chained_filter.yaml>`_ for a single interface:

.. literalinclude:: ../test/config/test_chained_filter.yaml
   :language: yaml

or for `multiple interfaces <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/chained_filter_controller/test/config/test_multiple_chained_filter.yaml>`_:

.. literalinclude:: ../test/config/test_multiple_chained_filter.yaml
   :language: yaml
