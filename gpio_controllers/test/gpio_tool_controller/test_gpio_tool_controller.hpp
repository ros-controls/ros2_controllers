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

#ifndef GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_
#define GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_

#include <gmock/gmock.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "gpio_controllers/gpio_tool_controller.hpp"

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableGpioToolController : public gpio_tool_controller::GpioToolController
{
  FRIEND_TEST(GpioToolControllerTest, AllParamsSetSuccess);
  FRIEND_TEST(GpioToolControllerTest, AllParamNotSetFailure);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = gpio_tool_controller::GpioToolController::on_configure(previous_state);
    return ret;
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class IOGripperControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(
    const std::string controller_name = "test_gpio_tool_controller",
    const std::vector<rclcpp::Parameter> & parameters = {})
  {
    RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "initializing controller");
    auto node_options = controller_->define_custom_node_options();
    node_options.parameter_overrides(parameters);

    ASSERT_EQ(
      controller_->init(controller_name, "", 0, "", node_options),
      controller_interface::return_type::OK);
    RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "initialized successfully");
  }

  void setup_parameters()
  {
    controller_->get_node()->set_parameter({"use_action", true});
    controller_->get_node()->set_parameter({"timeout", 5.0});
    controller_->get_node()->set_parameter({"tolerance", 0.00001});
    controller_->get_node()->set_parameter(
      {"engaged_joints", std::vector<std::string>{"gripper_clamp_jaw"}});

    // Disengaged state
    controller_->get_node()->set_parameter({"disengaged.name", std::string("open")});
    controller_->get_node()->set_parameter({"disengaged.joint_states", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.interfaces", std::vector<std::string>{"Open_valve", "Close_valve"}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.interfaces", std::vector<std::string>{"Opened_signal", "Closed_signal"}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.values", std::vector<double>{1.0}});

    // Possible engaged states
    controller_->get_node()->set_parameter(
      {"possible_engaged_states", std::vector<std::string>{"close_empty", "close_full"}});

    // Engaged state
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.command.interfaces", std::vector<std::string>{"Close_valve", "Open_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.command.values", std::vector<double>{1.0, 0.0}});

    // Engaged states: close_empty
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.joint_states", std::vector<double>{0.16}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_values", std::vector<double>{1.0}});

    // Engaged states: close_full
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.joint_states", std::vector<double>{0.08}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_values", std::vector<double>{1.0}});

    // Configurations
    controller_->get_node()->set_parameter({"configurations", std::vector<std::string>{}});
    controller_->get_node()->set_parameter({"configuration_joints", std::vector<std::string>{}});

    // Tool specific sensors
    controller_->get_node()->set_parameter({"tool_specific_sensors", std::vector<std::string>{}});
  }

protected:
  const std::vector<std::string> possible_engaged_states = {"close_empty", "close_full"};

  // Test related parameters
  std::unique_ptr<TestableGpioToolController> controller_;
};

class GpioToolControllerTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};
#endif  // GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_
