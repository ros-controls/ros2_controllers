// Copyright (c) 2021, PickNik, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

const std::string & get_urdf()
{
  static const std::string urdf = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <origin rpy="-1.57079632679 0 0" xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="joint2" type="revolute">
    <origin rpy="1.57079632679 0 0" xyz="0 0 0.9"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="tool_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 1"/>
    <parent link="link2"/>
    <child link="tool_link"/>
  </joint>
  <link name="tool_link">
  </link>

  <ros2_control name="TestActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_hardware_components/TestSingleJointActuator</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestSensorHardware" type="sensor">
    <hardware>
      <plugin>test_hardware_components/TestForceTorqueSensor</plugin>
    </hardware>
    <sensor name="sensor1">
      <state_interface name="fx"/>
      <state_interface name="fy"/>
      <state_interface name="fz"/>
      <state_interface name="tx"/>
      <state_interface name="ty"/>
      <state_interface name="tz"/>
    </sensor>
  </ros2_control>
  <ros2_control name="TestSystemHardware" type="system">
    <hardware>
      <plugin>test_hardware_components/TestTwoJointSystem</plugin>
    </hardware>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="joint3">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";
  return urdf;
}

TEST(TestLoadAdmittanceController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(get_urdf()), executor,
    "test_controller_manager", "", options);

  const std::string test_file_path = std::string(TEST_FILES_DIRECTORY) + "/test_params.yaml";

  cm.set_parameter({"load_admittance_controller.params_file", test_file_path});
  cm.set_parameter(
    {"load_admittance_controller.type", "admittance_controller/AdmittanceController"});

  ASSERT_NE(cm.load_controller("load_admittance_controller"), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  std::vector<std::string> args;
  for (int i = 0; i < argc; ++i)
  {
    args.push_back(argv[i]);
  }
  args.push_back("--ros-args");
  args.push_back("-p");
  args.push_back("robot_description:=" + get_urdf());

  std::vector<char *> custom_argv;
  for (const auto & arg : args)
  {
    custom_argv.push_back(const_cast<char *>(arg.c_str()));
  }
  int custom_argc = custom_argv.size();

  rclcpp::init(custom_argc, custom_argv.data());
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
