admittance_controller
======================

The controller uses admittance dynamics to calculate robot movements caused by an external force.
The standard admittance control law: F = M\*a + D\*v + K\*(x_d - x) is used.

The paramters are adjustable using ``dynamic_reconfigure`` functionality and rqt-plugin.

The controller has following parameters:

  - ``joints`` - names of joints to control the robot.
  - ``command_interfaces`` - interfaces uses for commanding the robot, e.g., position and/or velocity.
  - ``state_interfaces`` - interfaces to get robot's current state. Currently only position is supported.
  - ``ft_sensor_name`` - name of the sensor providing force-torque measurements. The controller uses `ForceTorqueSensor semantic component <https://github.com/destogl/ros2_control/blob/add-semantic-components/controller_interface/include/semantic_components/force_torque_sensor.hpp>`_ which defines standard interface names for this measurements, e.g., <sensor_name>/force.x.

  - ``IK.base`` - base link of the IK, usually the first fixed link on a robot, e.g., ``base_link``.
  - ``IK.tip`` - end of the IK, usually flansh of the robot, e.g., ``tool0``.
  - ``IK.group_name`` - name of the IK group defined when using a IK plugin (not implemented yet).

  - ``controller_frame`` - frame where all input and measured values will be transformed to do the calculation of admittance equation.
  - ``endeffector_frame`` - frame of the endeffector, where force is influencing the robot and which pose is manipulated.
  - ``fixed_world_frame`` - frame where gravity vector is oriented in ``-z`` direction.
  - ``sensor_frame`` - frame where force-torque values are measured.

  - ``gravity_compensation.masses`` - list of masses of robot parts to subtract their gravity influence from force/torque measurements.
  - ``gravity_compensation.center_of_masses`` - list of distances for the corresponding mass from ``sensor_frame`` (one dimensional).

  - ``admittance.selected_axes.[x|y|z|rx|ry|rz]`` - boolean value to enable admittance control for specific degree of freedom in ``controller_frame``.
  - ``admittance.mass.[x|y|z|rx|ry|rz]`` - six virtual mass (M) values for degrees of freedom.
  - ``admittance.damping.[x|y|z|rx|ry|rz]`` - six virtual damping (D) values for degrees of freedom.
  - ``admittance.stiffness.[x|y|z|rx|ry|rz]`` - six virtual stiffness values (K) for degrees of freedom.


Publishers:

  - ``state`` - state message of the controller as ``control_msgs/msg/AdmittanceControllerState``:

      geometry_msgs/WrenchStamped input_force_command  #commanded input force for the controller
      geometry_msgs/PoseStamped input_pose_command     #commanded input pose for the controller

      geometry_msgs/WrenchStamped input_force_transformed  # input force transformed into control frame
      geometry_msgs/PoseStamped input_pose_transformed     # input pose transformed into control frame

      geometry_msgs/WrenchStamped measured_force           # measured force from the sensor (sensor frame)
      geometry_msgs/WrenchStamped measured_force_filtered  # measured force after low-pass and gravity compensation filters in sensor frame
      geometry_msgs/WrenchStamped measured_force_transformed  # transformed measured force to control frame

      geometry_msgs/PoseStamped goal_pose_command  # goal pose to be commanded

      string[] joint_names
      trajectory_msgs/JointTrajectoryPoint desired_joint_states  # result of IK from goal_pose_command
      trajectory_msgs/JointTrajectoryPoint actual_joint_states   # read from the hardware
      trajectory_msgs/JointTrajectoryPoint error_joint_state


Subscribers:

  - ``input_pose_command`` - desired pose as ``geometry_msgs/msg/PoseStamped``
