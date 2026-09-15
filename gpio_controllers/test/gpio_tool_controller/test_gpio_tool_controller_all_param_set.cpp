// Copyright (c) 2025, b»robotized by Stogl Robotics
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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

// Test setting all params and getting success
TEST_F(GpioToolControllerTest, AllParamsSetSuccess)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();

  // configure success.
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  RCLCPP_INFO(rclcpp::get_logger("GpioToolControllerTest"), "Setup parameters successfully");
}

// ---------------------------------------------------------------------------
// disengaged.command interfaces/values size mismatch → FAILURE
// Covers the check_parameter_pairs() call for the disengaged command block
// (gpio_tool_controller.cpp lines 166-170).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, DisengagedCommandInterfacesValuesMismatch)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  // 2 interfaces but only 1 value → size mismatch
  controller_->get_node()->set_parameter(
    {"disengaged.command.interfaces", std::vector<std::string>{"Open_valve", "Close_valve"}});
  controller_->get_node()->set_parameter({"disengaged.command.values", std::vector<double>{1.0}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// engaged.command interfaces/values size mismatch → FAILURE
// Covers the check_parameter_pairs() call for the engaged command block
// (gpio_tool_controller.cpp lines 185-190).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, EngagedCommandInterfacesValuesMismatch)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  // 2 interfaces but 3 values → size mismatch
  controller_->get_node()->set_parameter(
    {"engaged.command.interfaces", std::vector<std::string>{"Close_valve", "Open_valve"}});
  controller_->get_node()->set_parameter(
    {"engaged.command.values", std::vector<double>{1.0, 0.0, 0.5}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// engaged.states.<name>.interfaces/values size mismatch → FAILURE
// Covers the per-state check_parameter_pairs() inside the possible_engaged_states
// loop (gpio_tool_controller.cpp lines 197-198).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, EngagedStateInterfacesValuesMismatch)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  // close_empty: 2 interfaces but only 1 value → size mismatch
  controller_->get_node()->set_parameter(
    {"engaged.states.close_empty.interfaces",
     std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
  controller_->get_node()->set_parameter(
    {"engaged.states.close_empty.values", std::vector<double>{1.0}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// configuration_setup.<name>.command_interfaces/values size mismatch → FAILURE
// Covers the per-configuration check_parameter_pairs() inside the configurations
// loop (gpio_tool_controller.cpp lines 241-243).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, ConfigurationCommandInterfacesValuesMismatch)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter(
       "configuration_joints", std::vector<std::string>{"gripper_distance_joint"})});
  setup_parameters_with_config();
  // narrow_objects: 2 command interfaces but only 1 value → size mismatch
  controller_->get_node()->set_parameter(
    {"configuration_setup.narrow_objects.command_interfaces",
     std::vector<std::string>{"Narrow_Configuration_Cmd", "Wide_Configuration_Cmd"}});
  controller_->get_node()->set_parameter(
    {"configuration_setup.narrow_objects.command_values", std::vector<double>{1.0}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// tool_specific_sensors entry with empty interface name → FAILURE
// Covers the empty-interface guard inside the sensors_interfaces loop
// (gpio_tool_controller.cpp lines 270-276).
// sensors_interfaces.<name>.interface defaults to "" when not explicitly set,
// which directly triggers the fatal check.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerTest, ToolSpecificSensorEmptyInterfaceName)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  // Declare a sensor; its interface defaults to "" → triggers the fatal check
  controller_->get_node()->set_parameter(
    {"tool_specific_sensors", std::vector<std::string>{"pressure_sensor"}});
  // sensors_interfaces.pressure_sensor.interface is auto-declared with default "" → FAILURE

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

// Test not setting the one param and getting failure
TEST_F(GpioToolControllerTest, AllParamNotSetFailure)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});

  // Engaged joint size not equal to Disengaged joint states size
  controller_->get_node()->set_parameter({"engaged_joints", std::vector<std::string>{}});
  controller_->get_node()->set_parameter({"disengaged.joint_states", std::vector<double>{0.0}});
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);

  controller_->get_node()->set_parameter(
    {"engaged_joints", std::vector<std::string>{"gripper_clamp_jaw"}});

  // engaged.close_empty state
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);

  controller_->get_node()->set_parameter(
    {"engaged.states.close_empty.joint_states", std::vector<double>{0.16}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);

  // engaged.close_full state
  controller_->get_node()->set_parameter(
    {"engaged.states.close_full.joint_states", std::vector<double>{0.08}});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
