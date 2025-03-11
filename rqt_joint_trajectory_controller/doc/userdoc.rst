:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/rqt_joint_trajectory_controller/doc/userdoc.rst

.. _rqt_joint_trajectory_controller_userdoc:

rqt_joint_trajectory_controller
===============================

The rqt_joint_trajectory_controller provides an intuitive graphical way to test different joint positions and trajectories without having to manually construct complex trajectory messages or use command line interfaces.

.. image:: rqt_joint_trajectory_controller.png
  :width: 400
  :alt: rqt_joint_trajectory_controller

The interface allows you to:

* Select the controller manager namespace and controller from dropdown menus.
* Adjust target positions for joints (joint1 and joint2) using interactive sliders.
* Fine-tune joint positions with precise numerical inputs.
* Control the motion speed using the speed scaling slider.
* Activate the trajectory execution with the central power button.
* Visualize current joint configurations in real-time.
