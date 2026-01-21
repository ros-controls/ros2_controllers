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
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "gpio_controllers/gpio_command_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
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

class TestableGpioController : public gpio_controllers::GpioCommandController
{
public:
  using gpio_controllers::GpioCommandController::assign_interfaces;
};

TEST(TestLoadGpioCommandController, UpdateBoolGpioInterfaces)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions node_options;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("gpios", std::vector<std::string>{"gpio1"});
  params.emplace_back("command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig_out_1"});
  params.emplace_back("state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig_in_1"});
  node_options.parameter_overrides(params);

  auto controller = std::make_shared<TestableGpioController>();

  controller_interface::ControllerInterfaceParams init_params;
  init_params.controller_name = "test_gpio_controller";
  init_params.node_options = node_options;

  ASSERT_EQ(controller->init(init_params), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  double dummy_double_value = 0.0;

  auto cmd_intf = std::make_shared<hardware_interface::CommandInterface>(
    "gpio1", "dig_out_1", &dummy_double_value);
  std::vector<hardware_interface::LoanedCommandInterface> cmd_loaned;
  cmd_loaned.emplace_back(cmd_intf);

  auto state_intf =
    std::make_shared<hardware_interface::StateInterface>("gpio1", "dig_in_1", &dummy_double_value);
  std::vector<hardware_interface::LoanedStateInterface> state_loaned;
  state_loaned.emplace_back(state_intf);

  controller->assign_interfaces(std::move(cmd_loaned), std::move(state_loaned));

  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // This verifies that the controller no longer crashes on update
  EXPECT_NO_THROW(controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)));

  rclcpp::shutdown();
}
