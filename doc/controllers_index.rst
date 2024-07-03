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
   :glob:

   *


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
   Parallel Gripper Controller <../parallel_gripper_controller/doc/userdoc.rst>
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


Common Controller Parameters
****************************

Every controller and broadcaster has a few common parameters. They are optional, but if needed they have to be set before ``onConfigure`` transition to ``inactive`` state, see `lifecycle documents <https://design.ros2.org/articles/node_lifecycle.html>`__. Once the controllers are already loaded, this transition is done using the service ``configure_controller`` of the controller_manager.

* ``update_rate``: An unsigned integer parameter representing the rate at which every controller/broadcaster runs its update cycle. When unspecified, they run at the same frequency as the controller_manager.
* ``is_async``: A boolean parameter that is needed to specify if the controller update needs to run asynchronously.
