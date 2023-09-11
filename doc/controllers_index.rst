:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/controllers_index.rst

.. _controllers:

#################
ros2_controllers
#################

`GitHub Repository <https://github.com/ros-controls/ros2_controllers>`_


Nomenclature
************

The ros2_control framework uses namespaces to sort controller according to the type of command interface they are using.
The controllers are using `common hardware interface definitions`_.
The controllers' namespaces are commanding the following command interface types:

  - ``position_controllers``: ``hardware_interface::HW_IF_POSITION``
  - ``velocity_controller``: ``hardware_interface::HW_IF_VELOCITY``
  - ``effort_controllers``: ``hardware_interface::HW_IF_ACCELERATION``
  - ``effort_controllers``: ``hardware_interface::HW_IF_EFFORT``
  - ...

.. _common hardware interface definitions: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp



Guidelines and Best Practices
*****************************

.. toctree::
   :titlesonly:
   :glob:

   *


Available Controllers
*********************

.. toctree::
   :titlesonly:

   Ackermann Steering Controller <../ackermann_steering_controller/doc/userdoc.rst>
   Admittance Controller <../admittance_controller/doc/userdoc.rst>
   Bicycle Steering Controller <../bicycle_steering_controller/doc/userdoc.rst>
   Differential Drive Controller <../diff_drive_controller/doc/userdoc.rst>
   Effort Controllers <../effort_controllers/doc/userdoc.rst>
   Forward Command Controller <../forward_command_controller/doc/userdoc.rst>
   Gripper Controller <../gripper_controllers/doc/userdoc.rst>
   Joint Trajectory Controller <../joint_trajectory_controller/doc/userdoc.rst>
   Position Controllers <../position_controllers/doc/userdoc.rst>
   Steering Controllers Library <../steering_controllers_library/doc/userdoc.rst>
   Tricycle Controller <../tricycle_controller/doc/userdoc.rst>
   Tricycle Steering Controller <../tricycle_steering_controller/doc/userdoc.rst>
   Velocity Controllers <../velocity_controllers/doc/userdoc.rst>


Available Broadcasters
**********************

.. toctree::
   :titlesonly:

   Force Torque Sensor Broadcaster <../force_torque_sensor_broadcaster/doc/userdoc.rst>
   Imu Sensor Broadcaster <../imu_sensor_broadcaster/doc/userdoc.rst>
   Joint State Broadcaster <../joint_state_broadcaster/doc/userdoc.rst>
   Range Sensor Broadcaster <../range_sensor_broadcaster/doc/userdoc.rst>
