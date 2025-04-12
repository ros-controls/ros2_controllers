:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/effort_controllers/doc/userdoc.rst

.. _effort_controllers_userdoc:

effort_controllers
==================

This is a collection of controllers that work using the "effort" joint command interface but may accept different joint-level commands at the controller level, e.g. controlling the effort on a certain joint to achieve a set position.

The package contains the following controllers:

effort_controllers/JointGroupEffortController
-------------------------------------------------

This is specialization of the :ref:`forward_command_controller <forward_command_controller_userdoc>` that works using the "effort" joint interface.


ROS 2 interface of the controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Topics
,,,,,,,,,,,,,,,,,,

~/commands (input topic) [std_msgs::msg::Float64MultiArray]
  Joints' effort commands


Parameters
,,,,,,,,,,,,,,,,,,
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
