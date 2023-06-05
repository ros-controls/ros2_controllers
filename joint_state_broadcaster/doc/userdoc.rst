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
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

map_interface_to_joint_state
  Optional parameter (map) providing mapping between custom interface names to standard fields in ``joint_states`` message.
  Usecases:

    1. Hydraulics robots where feedback and commanded values often have an offset and reliance on open-loop control is common.
       Typically one would map both values in separate interfaces in the framework.
       To visualize those data multiple joint_state_broadcaster instances and robot_state_publishers would be used to visualize both values in RViz.

    1. A robot provides multiple measuring techniques for its joint values which results in slightly different values.
       Typically one would use separate interface for providing those values in the framework.
       Using multiple joint_state_broadcaster instances we could publish and show both in RViz.

  Format (each line is optional):

  .. code-block:: yaml

      map_interface_to_joint_state:
        position: <custom_interface>
        velocity: <custom_interface>
        effort: <custom_interface>


  Examples:

  .. code-block:: yaml

      map_interface_to_joint_state:
        position: kf_estimated_position


  .. code-block:: yaml

      map_interface_to_joint_state:
        velocity: derived_velocity
        effort: derived_effort


  .. code-block:: yaml

      map_interface_to_joint_state:
        effort: torque_sensor


  .. code-block:: yaml

      map_interface_to_joint_state:
        effort: current_sensor

Full list of parameters:

.. generate_parameter_library_details:: ../src/joint_state_broadcaster_parameters.yaml
