:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/controllers_index.rst

.. _controllers:

#################
ros2_controllers
#################

Commonly used and generalized controllers for ros2_control framework that are ready to use with many robots, `MoveIt2 <https://moveit.picknik.ai/main/index.html>`_ and `Nav2 <https://nav2.org/>`_.

`Link to GitHub Repository <https://github.com/ros-controls/ros2_controllers>`_


Guidelines and Best Practices
*****************************

.. toctree::
   :titlesonly:

   mobile_robot_kinematics.rst
   writing_new_controller.rst


Controllers for Wheeled Mobile Robots
*************************************

.. toctree::
   :titlesonly:

   Differential Drive Controller <../diff_drive_controller/doc/userdoc.rst>
   Steering Controllers Library <../steering_controllers_library/doc/userdoc.rst>
   Tricycle Controller <../tricycle_controller/doc/userdoc.rst>

Controllers for Manipulators and Other Robots
*********************************************

The controllers are using `common hardware interface definitions`_, and may use namespaces depending on the following command interface types:

  - ``position_controllers``: ``hardware_interface::HW_IF_POSITION``
  - ``velocity_controller``: ``hardware_interface::HW_IF_VELOCITY``
  - ``effort_controllers``: ``hardware_interface::HW_IF_ACCELERATION``
  - ``effort_controllers``: ``hardware_interface::HW_IF_EFFORT``

.. _common hardware interface definitions: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp


.. toctree::
   :titlesonly:

   Admittance Controller <../admittance_controller/doc/userdoc.rst>
   Effort Controllers <../effort_controllers/doc/userdoc.rst>
   Forward Command Controller <../forward_command_controller/doc/userdoc.rst>
   Gripper Controller <../gripper_controllers/doc/userdoc.rst>
   Joint Trajectory Controller <../joint_trajectory_controller/doc/userdoc.rst>
   PID Controller <../pid_controller/doc/userdoc.rst>
   Position Controllers <../position_controllers/doc/userdoc.rst>
   Velocity Controllers <../velocity_controllers/doc/userdoc.rst>


Broadcasters
**********************

Broadcasters are used to publish sensor data from hardware components to ROS topics.
In the sense of ros2_control, broadcasters are still controllers using the same controller interface as the other controllers above.

.. toctree::
   :titlesonly:

   Force Torque Sensor Broadcaster <../force_torque_sensor_broadcaster/doc/userdoc.rst>
   IMU Sensor Broadcaster <../imu_sensor_broadcaster/doc/userdoc.rst>
   Joint State Broadcaster <../joint_state_broadcaster/doc/userdoc.rst>
   Range Sensor Broadcaster <../range_sensor_broadcaster/doc/userdoc.rst>
   Pose Broadcaster <../pose_broadcaster/doc/userdoc.rst>
