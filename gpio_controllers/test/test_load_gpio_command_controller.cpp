// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

const auto urdf_bool = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="BoolGpioBot">
  <link name="world"/>
  <link name="base_link"/>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <ros2_control name="BoolGpioBot" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
    <gpio name="gpio1">
      <command_interface name="dig_out_1" data_type="bool"/>
      <state_interface name="dig_in_1" data_type="bool"/>
    </gpio>
  </ros2_control>
</robot>
)";

TEST(TestLoadGpioCommandController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller("test_gpio_command_controller", "gpio_controllers/GpioCommandController"));

  rclcpp::shutdown();
}

TEST(TestLoadGpioCommandController, load_controller_bool_crash)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  rclcpp::NodeOptions node_options;
  node_options.arguments({
    "--ros-args",
    "-p", "test_gpio_command_controller_2.gpios:=['gpio1']",
    "-p", "test_gpio_command_controller_2.command_interfaces.gpio1.interfaces:=['dig_out_1']",
    "-p", "test_gpio_command_controller_2.state_interfaces.gpio1.interfaces:=['dig_in_1']"
  });

  controller_manager::ControllerManager cm(
    executor, 
    urdf_bool, 
    true, 
    "test_controller_manager", 
    "", 
    node_options
  );

  ASSERT_NO_THROW(
      cm.load_controller("test_gpio_command_controller_2", "gpio_controllers/GpioCommandController"));

  cm.configure_controller("test_gpio_command_controller_2");

  cm.switch_controller(
      {"test_gpio_command_controller_2"}, 
      {}, 
      controller_manager_msgs::srv::SwitchController::Request::STRICT, 
      true, 
      rclcpp::Duration(0, 0)
  );

  cm.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  rclcpp::shutdown();
}