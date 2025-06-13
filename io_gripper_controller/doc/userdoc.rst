:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/io_gripper_controller/doc/userdoc.rst

.. _io_gripper_controller_userdoc:

io_gripper_controller
=============================

The IO Gripper Controller provides implementation to control the grippers that are commanded using IOs.
This is often the case for pneumatic gripper in the industy, that can range from simple parallel gripper up to custom, multi-dof grippers for manipulating specific parts.
 It provides functionalities like open, close and reconfigure which can be used either though action server or service server and also publishes gripper's joint values if any and provides output for all gripper's command and state interfaces.

Description of controller's interfaces
---------------------------------------

- ``joint_states`` [``sensor_msgs::msg::JointState``]: Publishes the state of gripper's open/close joint if any and configuration joints that might influece the geometry and kinematics of the gripper.
- ``dynamic_interfaces`` [``control_msgs::msg::DynamicInterfaceValues``]: Publishes all command and state interface of the IOs and sensors of gripper.


When preemting acitons make sure that those are defined fully in the controller's parameter.
This means that you shoul make sure that the actuators for *engage* are disabled when *disengage* is called and vice versa.
Otherwise it can happen that two actuators acting "against each other" are turned on at the same time which can lead to unexpected behavior and even damage of the tool.

Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

This controller adds the following parameters:

.. generate_parameter_library_details:: ../src/io_gripper_controller.yaml


Example parameters
....................

.. code-block:: yaml

  io_gripper_controller:

    ros__parameters:

      use_action: true

      open_close_joints: [gripper_clamp_jaw]

      open:
        joint_states: [0.0]
        set_before_command:
          high: [Release_Break_valve]
          low: []
        command:
          high: [Open_valve]
          low: [Close_valve]
        state:
          high: [Opened_signal]
          low: [Closed_signal]
        set_after_command:
          high: []
          low: [Release_Break_valve]

      possible_closed_states: ['empty_close', 'full_close']

      close:
        set_before_command:
          high: [Release_Break_valve]
          low: []
        command:
          high: [Close_valve]
          low: [Open_valve]
        state:
          empty_close:
            joint_states: [0.16]
            high: [Closed_signal]
            low: [Part_Grasped_signal]
            set_after_command_high: []
            set_after_command_low: [Release_Break_valve]
          full_close:
            joint_states: [0.08]
            high: [Part_Grasped_signal]
            low: [Closed_signal]
            set_after_command_high: []
            set_after_command_low: [Release_Break_valve]

      configurations: ["narrow_objects", "wide_objects"]
      configuration_joints: [gripper_gripper_distance_joint]

      configuration_setup:
        narrow_objects:
          joint_states: [0.1]
          command_high: [Narrow_Configuration_Cmd]
          command_low: [Wide_Configuration_Cmd]
          state_high: [Narrow_Configuraiton_Signal]
          state_low: [Wide_Configuration_Signal]
        wide_objects:
          joint_states: [0.2]
          command_high: [Wide_Configuration_Cmd]
          command_low: [Narrow_Configuration_Cmd]
          state_high: [Wide_Configuration_Signal]
          state_low: [Narrow_Configuraiton_Signal]

      gripper_specific_sensors: ["part_sensor_top", "part_sensor"]
      sensors_interfaces:
        part_sensor_top:
          input: "Part_Sensor_Top_signal"
        part_sensor:
          input: "Part_Grasped_signal"
