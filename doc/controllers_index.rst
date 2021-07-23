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
  - ``effort_controllers``: ``hardware_interface::HW_IF_EFFORT``
  - ...


Controllers
***********

The following standard controllers are implemented:

  - `Joint Trajectory Controller <joint_trajectory_controller/docs/index.rst>`_ - provided a list of waypoints or target point defined with position, velocity and acceleration, the controller interpolates joint trajectories through it.
  - ... <the list is not complete> ...

.. _common hardware interface definitions: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp


Guidelines and Best Practices
*****************************

.. toctree::
   :titlesonly:
   :glob:

   *


Available Controllers
=====================

.. toctree::
   :titlesonly:

   Differential Drive <../diff_drive_controller/doc/userdoc.rst>
   Forward Command <../forward_command_controller/doc/userdoc.rst>
   Joint Trajectory <../joint_trajectory_controller/doc/userdoc.rst>
   Position Controllers <../position_controllers/doc/userdoc.rst>
   Velocity Controllers <../velocity_controllers/doc/userdoc.rst>
   Effort Controllers <../effort_controllers/doc/userdoc.rst>


Available Broadcasters
======================

.. toctree::
   :titlesonly:

   Joint State Broadcaster <../joint_state_broadcaster/doc/userdoc.rst>
   Imu Sensor Broadcaster <../imu_sensor_broadcaster/doc/userdoc.rst>
