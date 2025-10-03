:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/forward_command_controller/doc/userdoc.rst

.. _forward_command_controller_userdoc:

forward_command_controller
==========================

A selection of controllers that forward commands of different types.

forward_command_controller and multi_interface_forward_command_controller
#########################################################################
Both controllers forward ``std_msgs::msg::Float64MultiArray`` to a set of interfaces, which can be parameterized as follows: While ``forward_command_controller/ForwardCommandController`` only claims a single interface type per joint (``joint[i] + "/" + interface_name``), the ``forward_command_controller/MultiInterfaceForwardCommandController`` claims the combination of all interfaces specified in the ``interface_names`` parameter (``joint[i] + "/" + interface_names[j]``).

Hardware interface type
-----------------------

This controller can be used for every type of command interface, not only limited to joints.


ROS 2 interface of the controller
---------------------------------

Topics
^^^^^^^

~/commands (input topic) [std_msgs::msg::Float64MultiArray]
  Target joint commands

Parameters
^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

   .. tabs::

      .. group-tab:: forward_command_controller

        .. generate_parameter_library_details:: ../src/forward_command_controller_parameters.yaml

      .. group-tab:: multi_interface_forward_command_controller

        .. generate_parameter_library_details:: ../src/multi_interface_forward_command_controller_parameters.yaml
