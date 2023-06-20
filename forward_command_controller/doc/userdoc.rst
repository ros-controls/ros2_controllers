:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/forward_command_controller/doc/userdoc.rst

.. _forward_command_controller_userdoc:

forward_command_controller
==========================

This is a base class implementing a feedforward controller. Specific implementations can be found in:

* :ref:`position_controllers_userdoc`
* :ref:`velocity_controllers_userdoc`
* :ref:`effort_controllers_userdoc`

Hardware interface type
-----------------------

This controller can be used for every type of command interface.

Parameters
------------

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

forward_command_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. generate_parameter_library_details:: ../src/forward_command_controller_parameters.yaml

multi_interface_forward_command_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. generate_parameter_library_details:: ../src/multi_interface_forward_command_controller_parameters.yaml
