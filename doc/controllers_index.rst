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
   Mecanum Drive Controllers <../mecanum_drive_controller/doc/userdoc.rst>
   Omni Wheel Drive Controller <../omni_wheel_drive_controller/doc/userdoc.rst>
   Steering Controllers Library <../steering_controllers_library/doc/userdoc.rst>
   Tricycle Controller <../tricycle_controller/doc/userdoc.rst>

Controllers for Manipulators and Other Robots
*********************************************

.. toctree::
   :titlesonly:

   Admittance Controller <../admittance_controller/doc/userdoc.rst>
   Effort Controllers <../effort_controllers/doc/userdoc.rst>
   Forward Command Controller <../forward_command_controller/doc/userdoc.rst>
   Joint Trajectory Controller <../joint_trajectory_controller/doc/userdoc.rst>
   Parallel Gripper Controller <../parallel_gripper_controller/doc/userdoc.rst>
   PID Controller <../pid_controller/doc/userdoc.rst>
   Position Controllers <../position_controllers/doc/userdoc.rst>
   Velocity Controllers <../velocity_controllers/doc/userdoc.rst>
   Gpio Command Controller <../gpio_controllers/doc/userdoc.rst>
   Motion Primitive Controller <../motion_primitives_controllers/userdoc.rst>

Broadcasters
**********************

Broadcasters are used to publish sensor data from hardware components to ROS topics.
In the sense of ros2_control, broadcasters are still controllers using the same controller interface as the other controllers above.

.. toctree::
   :titlesonly:

   Force Torque Sensor Broadcaster <../force_torque_sensor_broadcaster/doc/userdoc.rst>
   IMU Sensor Broadcaster <../imu_sensor_broadcaster/doc/userdoc.rst>
   Interfaces State Broadcaster <../interfaces_state_broadcaster/doc/userdoc.rst>
   Joint State Broadcaster <../joint_state_broadcaster/doc/userdoc.rst>
   Range Sensor Broadcaster <../range_sensor_broadcaster/doc/userdoc.rst>
   Pose Broadcaster <../pose_broadcaster/doc/userdoc.rst>
   GPS Sensor Broadcaster <../gps_sensor_broadcaster/doc/userdoc.rst>

Filters
**********************

Chainable controllers for filtering of state interfaces. They export the filtered values as state interfaces, which can be used by other controllers or broadcasters, and don't publish to ROS topics.

.. toctree::
   :titlesonly:

   Chained Filter Controller <../chained_filter_controller/doc/userdoc.rst>

Common Controller Parameters
****************************

Every controller and broadcaster has a few common parameters. They are optional, but if needed they have to be set before ``onConfigure`` transition to ``inactive`` state, see `lifecycle documents <https://design.ros2.org/articles/node_lifecycle.html>`__. Once the controllers are already loaded, this transition is done using the service ``configure_controller`` of the controller_manager.

* ``update_rate``: An unsigned integer parameter representing the rate at which every controller/broadcaster runs its update cycle. When unspecified, they run at the same frequency as the controller_manager.
* ``is_async``: A boolean parameter that is needed to specify if the controller update needs to run asynchronously.
