:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/position_controllers/doc/userdoc.rst

.. _position_controllers_userdoc:

position_controllers
=====================

This is a collection of controllers that work using the "position" joint command interface but may accept different joint-level commands at the controller level, e.g. controlling the position on a certain joint to achieve a set velocity.

The package contains the following controllers:

position_controllers/JointGroupPositionController
-------------------------------------------------

This is specialization of the :ref:`forward_command_controller <forward_command_controller_userdoc>` that works using the "position" joint interface.


ROS 2 interface of the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Topics
,,,,,,,,,,,,,,,,,,

~/commands (input topic) [std_msgs::msg::Float64MultiArray]
  Joints' position commands


Parameters
,,,,,,,,,,,,,,,,,,
This controller overrides the interface parameter from :ref:`forward_command_controller <forward_command_controller_userdoc>`, and the
``joints`` parameter is the only one that is required.

An example parameter file is given here

.. code-block:: yaml

  controller_manager:
    ros__parameters:
      update_rate: 100  # Hz

      position_controller:
        type: position_controllers/JointGroupPositionController

  position_controller:
    ros__parameters:
      joints:
        - slider_to_cart
