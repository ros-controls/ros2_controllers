:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/io_gripper_controller/doc/userdoc.rst

.. _io_gripper_controller_userdoc:

io_gripper_controller
=============================

The IO Gripper Controller provides implementation to control the gripper using IOs. It provides functionalities like open, close and reconfigure which can be used either though action server or service server and also publishes ``joint_states`` of gripper and also ``dynamic_interfaces`` for all command and state interfaces.

Description of controller's interfaces
---------------------------------------

- ``joint_states`` [``sensor_msgs::msg::JointState``]: Publishes the state of gripper joint and configuration joint
- ``dynamic_interfaces`` [``control_msgs::msg::DynamicInterfaceValues``]: Publishes all command and state interface of the IOs and sensors of gripper.


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
          high: [EL2008/Bremse_WQG7]
          low: []
        command:
          high: [EL2008/Greiferteil_Oeffnen_WQG1]
          low: [EL2008/Greiferteil_Schliessen_WQG2]
        state:
          high: [EL1008/Greifer_Geoeffnet_BG01]
          low: [EL1008/Greifer_Geschloschen_BG02]
        set_after_command:
          high: []
          low: [EL2008/Bremse_WQG7]

      possible_closed_states: ['empty_close', 'full_close']

      close:
        set_before_command:
          high: [EL2008/Bremse_WQG7]
          low: [EL2008/Greiferteil_Oeffnen_WQG1]
        command:
          high: [EL2008/Greiferteil_Schliessen_WQG2]
          low: [EL2008/Greiferteil_Oeffnen_WQG1]
        state:
          empty_close:
            joint_states: [0.08]
            high: [EL1008/Greifer_Geschloschen_BG02]
            low: [EL1008/Bauteilabfrage_BG06]
            set_after_command_high: [EL2008/Bremse_WQG7]
            set_after_command_low: [EL2008/Bremse_WQG7]
          full_close:
            joint_states: [0.08]
            high: [EL1008/Bauteilabfrage_BG06]
            low: [EL1008/Greifer_Geschloschen_BG02]
            set_after_command_high: [EL2008/Bremse_WQG7]
            set_after_command_low: [EL2008/Bremse_WQG7]

      configurations: ["stichmass_125", "stichmass_250"]
      configuration_joints: [gripper_gripper_distance_joint]

      configuration_setup:
        stichmass_125:
          joint_states: [0.125]
          command_high: [EL2008/Stichmass_125_WQG5]
          command_low: [EL2008/Stichmass_250_WQG6]
          state_high: [EL1008/Stichmass_125mm_BG03]
          state_low: [EL1008/Stichmass_250mm_BG04]
        stichmass_250:
          joint_states: [0.250]
          command_high: [EL2008/Stichmass_250_WQG6]
          command_low: [EL2008/Stichmass_125_WQG5]
          state_high: [EL1008/Stichmass_250mm_BG04]
          state_low: [EL1008/Stichmass_125mm_BG03]

      gripper_specific_sensors: ["hohenabfrage", "bauteilabfrage"]
      sensors_interfaces:
        hohenabfrage:
          input: "EL1008/Hohenabfrage_BG5"
        bauteilabfrage:
          input: "EL1008/Bauteilabfrage_BG06"
