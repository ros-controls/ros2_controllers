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
#include <unordered_map>
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
#include "control_msgs/msg/gpio_tool_transition.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "gpio_controllers/gpio_tool_controller.hpp"

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
constexpr auto NODE_FAILURE = controller_interface::CallbackReturn::FAILURE;
using GPIOToolTransition = control_msgs::msg::GPIOToolTransition;
using gpio_tool_controller::ToolAction;
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

  // --- State machine control helpers ---

  void start_disengaging()
  {
    current_tool_action_.store(ToolAction::DISENGAGING);
    current_tool_transition_.store(GPIOToolTransition::SET_BEFORE_COMMAND);
    transition_time_updated_.store(false);
  }

  void start_engaging()
  {
    current_tool_action_.store(ToolAction::ENGAGING);
    current_tool_transition_.store(GPIOToolTransition::SET_BEFORE_COMMAND);
    transition_time_updated_.store(false);
  }

  void start_reconfiguring(const std::string & config_name)
  {
    target_configuration_.set(config_name);
    current_tool_action_.store(ToolAction::RECONFIGURING);
    current_tool_transition_.store(GPIOToolTransition::SET_BEFORE_COMMAND);
    transition_time_updated_.store(false);
  }

  // --- State machine introspection helpers ---

  ToolAction get_current_action() const { return current_tool_action_.load(); }

  uint8_t get_current_transition() const { return current_tool_transition_.load(); }

  std::string get_current_state() const { return current_state_.get(); }

  std::string get_current_configuration() const { return current_configuration_.get(); }

  // --- Service / action request helpers (expose protected methods for testing) ---

  EngagingSrvType::Response call_process_engaging_request(
    const ToolAction & action, const std::string & name)
  {
    return process_engaging_request(action, name);
  }

  EngagingSrvType::Response call_process_reconfigure_request(const std::string & config_name)
  {
    return process_reconfigure_request(config_name);
  }

  // --- State forcing helpers for CANCELING / HALTED tests ---

  void force_canceling() { current_tool_action_.store(ToolAction::CANCELING); }

  void force_halted() { current_tool_transition_.store(GPIOToolTransition::HALTED); }

  void trigger_reset_halted() { reset_halted_.store(true); }

  // --- Inspection helpers ---

  const std::vector<double> & get_joint_states_values() const { return joint_states_values_; }

  bool has_action_server() const { return engaging_action_server_ != nullptr; }
  bool has_disengaged_service() const { return disengaged_service_ != nullptr; }
  bool has_engaged_service() const { return engaged_service_ != nullptr; }
  bool has_reconfigure_service() const { return reconfigure_tool_service_ != nullptr; }
  bool has_reset_service() const { return reset_service_ != nullptr; }
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

public:
  void SetUpController(
    const std::string controller_name = "test_gpio_tool_controller",
    const std::vector<rclcpp::Parameter> & parameters = {})
  {
    RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "initializing controller");
    auto node_options = controller_->define_custom_node_options();
    node_options.parameter_overrides(parameters);

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = controller_name;
    params.node_options = node_options;

    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
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

    // Configurations (disabled in base setup)
    controller_->get_node()->set_parameter({"configurations", std::vector<std::string>{}});
    controller_->get_node()->set_parameter({"configuration_joints", std::vector<std::string>{}});

    // Tool specific sensors
    controller_->get_node()->set_parameter({"tool_specific_sensors", std::vector<std::string>{}});
  }

  // Extended setup that enables reconfiguration with narrow/wide object configs
  void setup_parameters_with_config()
  {
    setup_parameters();
    controller_->get_node()->set_parameter(
      {"configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}});
    controller_->get_node()->set_parameter(
      {"configuration_joints", std::vector<std::string>{"gripper_distance_joint"}});

    controller_->get_node()->set_parameter(
      {"configuration_setup.narrow_objects.joint_states", std::vector<double>{0.1}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.narrow_objects.command_interfaces",
       std::vector<std::string>{"Narrow_Configuration_Cmd", "Wide_Configuration_Cmd"}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.narrow_objects.command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.narrow_objects.state_interfaces",
       std::vector<std::string>{"Narrow_Configuraiton_Signal", "Wide_Configuration_Signal"}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.narrow_objects.state_values", std::vector<double>{1.0, 0.0}});

    controller_->get_node()->set_parameter(
      {"configuration_setup.wide_objects.joint_states", std::vector<double>{0.2}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.wide_objects.command_interfaces",
       std::vector<std::string>{"Narrow_Configuration_Cmd", "Wide_Configuration_Cmd"}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.wide_objects.command_values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.wide_objects.state_interfaces",
       std::vector<std::string>{"Narrow_Configuraiton_Signal", "Wide_Configuration_Signal"}});
    controller_->get_node()->set_parameter(
      {"configuration_setup.wide_objects.state_values", std::vector<double>{0.0, 1.0}});
  }

  // Query the controller for its interface names (post-configure) and assign mock interfaces.
  // The controller uses index-based access, so interfaces must be given in the same order
  // as command_interface_configuration() / state_interface_configuration() return them.
  void SetupInterfaces()
  {
    auto cmd_cfg = controller_->command_interface_configuration();
    auto state_cfg = controller_->state_interface_configuration();

    // Resize backing storage once so pointers remain stable
    cmd_values_.resize(cmd_cfg.names.size(), 0.0);
    state_values_.resize(state_cfg.names.size(), 0.0);

    for (size_t i = 0; i < cmd_cfg.names.size(); ++i)
    {
      cmd_name_to_index_[cmd_cfg.names[i]] = i;
    }
    for (size_t i = 0; i < state_cfg.names.size(); ++i)
    {
      state_name_to_index_[state_cfg.names[i]] = i;
    }

    std::vector<hardware_interface::LoanedCommandInterface> cmd_loaned;
    std::vector<hardware_interface::LoanedStateInterface> state_loaned;

    for (size_t i = 0; i < cmd_cfg.names.size(); ++i)
    {
      // Using empty prefix so that the CommandInterface stores a pointer to cmd_values_[i].
      // The handle_name will be "/name" which is only used in log messages – not for data access.
      cmd_iface_ptrs_.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
          "", cmd_cfg.names[i], &cmd_values_[i]));
      cmd_loaned.emplace_back(cmd_iface_ptrs_.back(), nullptr);
    }
    for (size_t i = 0; i < state_cfg.names.size(); ++i)
    {
      state_iface_ptrs_.push_back(
        std::make_shared<hardware_interface::StateInterface>(
          "", state_cfg.names[i], &state_values_[i]));
      state_loaned.emplace_back(state_iface_ptrs_.back(), nullptr);
    }

    controller_->assign_interfaces(std::move(cmd_loaned), std::move(state_loaned));
  }

  // Set the state interface values that identify a known tool state so that on_activate()
  // can determine the current state without going to CANCELING.
  void SetInitialHardwareState(const std::string & state_name)
  {
    // Reset everything to zero first
    std::fill(state_values_.begin(), state_values_.end(), 0.0);

    if (state_name == "open")
    {
      SetStateValue("Opened_signal", 1.0);
      // Closed_signal stays 0.0, Break_Engaged stays 0.0
    }
    else if (state_name == "close_empty")
    {
      SetStateValue("Closed_signal", 1.0);
      // Part_Grasped_signal stays 0.0
    }
    else if (state_name == "close_full")
    {
      // Closed_signal stays 0.0
      SetStateValue("Part_Grasped_signal", 1.0);
    }
    else if (state_name == "narrow_objects")
    {
      // For config-enabled fixture
      SetStateValue("Opened_signal", 1.0);
      SetStateValue("Narrow_Configuraiton_Signal", 1.0);
    }
    else if (state_name == "wide_objects")
    {
      SetStateValue("Opened_signal", 1.0);
      SetStateValue("Wide_Configuration_Signal", 1.0);
    }
  }

  // --- Value access helpers ---

  double GetCmdValue(const std::string & name) const
  {
    return cmd_values_.at(cmd_name_to_index_.at(name));
  }

  void SetStateValue(const std::string & name, double value)
  {
    state_values_.at(state_name_to_index_.at(name)) = value;
  }

  double GetStateValue(const std::string & name) const
  {
    return state_values_.at(state_name_to_index_.at(name));
  }

  controller_interface::return_type UpdateController(double time_seconds = 0.0)
  {
    return controller_->update(
      rclcpp::Time(static_cast<int64_t>(time_seconds * 1e9), RCL_ROS_TIME),
      rclcpp::Duration::from_seconds(0.01));
  }

public:
  const std::vector<std::string> possible_engaged_states = {"close_empty", "close_full"};

  // Test related parameters
  std::unique_ptr<TestableGpioToolController> controller_;

  // Mock interface backing storage – must outlive both the loaned interfaces and all update() calls
  std::vector<double> cmd_values_;
  std::vector<double> state_values_;
  std::unordered_map<std::string, size_t> cmd_name_to_index_;
  std::unordered_map<std::string, size_t> state_name_to_index_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> cmd_iface_ptrs_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_iface_ptrs_;
};

class GpioToolControllerTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerOpenTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerCloseTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerRequestTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerReconfigureTest
: public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerLifecycleTest
: public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerCancelingTest
: public IOGripperControllerFixture<TestableGpioToolController>
{
};

class GpioToolControllerServiceModeTest
: public IOGripperControllerFixture<TestableGpioToolController>
{
};

#endif  // GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_
