:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_state_broadcaster/doc/userdoc.rst

.. _joint_state_broadcaster_userdoc:

joint_state_broadcaster
=======================

The broadcaster reads all state interfaces and reports them on ``/joint_states`` and ``/dynamic_joint_states``.

Commands
--------

Broadcasters are not real controllers, and therefore take no commands.

Hardware interface type
-----------------------

By default, all available *joint state interfaces* are used, unless configured otherwise.
In the latter case, resulting interfaces is defined by a "matrix" of interfaces defined by the cross-product of the ``joints`` and ``interfaces`` parameters.
If some requested interfaces are missing, the controller will print a warning about that, but work for other interfaces.
If none of the requested interface are not defined, the controller returns error on activation.

Parameters
----------
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/joint_state_broadcaster/src/joint_state_broadcaster_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.


List of parameters
,,,,,,,,,,,,,,,,,,

.. generate_parameter_library_details::
  ../src/joint_state_broadcaster_parameters.yaml
  joint_state_broadcaster_parameter_context.yml


An example parameter file
,,,,,,,,,,,,,,,,,,,,,,,,,

.. generate_parameter_library_default::
  ../src/joint_state_broadcaster_parameters.yaml
