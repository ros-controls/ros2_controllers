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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gpio_controllers/gpio_command_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using CmdType = control_msgs::msg::DynamicJointState;
using StateType = control_msgs::msg::DynamicJointState;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;

class FriendGpioCommandController : public gpio_controllers::GpioCommandController
{
  FRIEND_TEST(GpioCommandControllerTestSuite, CommandSuccessTest);
  FRIEND_TEST(GpioCommandControllerTestSuite, CommandSuccessTestWithOnlyOneGpio);
  FRIEND_TEST(GpioCommandControllerTestSuite, CommandCallbackTest);
};

class GpioCommandControllerTestSuite : public ::testing::Test
{
public:
  GpioCommandControllerTestSuite() { rclcpp::init(0, nullptr); }
  ~GpioCommandControllerTestSuite() { rclcpp::shutdown(); }

  FriendGpioCommandController controller;

  const std::vector<std::string> gpio_names{"gpio1", "gpio2"};
  std::vector<double> gpio_commands{1.0, 0.0, 3.1};
  std::vector<double> gpio_states{1.0, 0.0, 3.1};

  CommandInterface gpio_1_1_dig_cmd{gpio_names[0], "dig.1", &gpio_commands[0]};
  CommandInterface gpio_1_2_dig_cmd{gpio_names[0], "dig.2", &gpio_commands[1]};
  CommandInterface gpio_2_ana_cmd{gpio_names[1], "ana.1", &gpio_commands[2]};

  StateInterface gpio_1_1_dig_state{gpio_names[0], "dig.1", &gpio_states[0]};
  StateInterface gpio_1_2_dig_state{gpio_names[0], "dig.2", &gpio_states[1]};
  StateInterface gpio_2_ana_state{gpio_names[1], "ana.1", &gpio_states[2]};
};

TEST_F(GpioCommandControllerTestSuite, GpiosParameterNotSet)
{
  const auto result = controller.init(
    "test_gpio_command_controller", "", 0, "", controller.define_custom_node_options());
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, GpiosParameterIsEmpty)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, GpioWithMissingGpioParams)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, ConfigureAndActivateParamsSuccess)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", gpio_names);
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(GpioCommandControllerTestSuite, ActivateWithWrongGpiosNamesFails)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio4"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio4.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, CommandSuccessTest)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 3.1);

  auto command_ptr = std::make_shared<CmdType>();
  control_msgs::msg::InterfaceValue inteface_value_gpio1;
  inteface_value_gpio1.interface_names = {"dig.1", "dig.2"};
  inteface_value_gpio1.values = {0.0, 1.0};
  control_msgs::msg::InterfaceValue inteface_value_gpio2;
  inteface_value_gpio2.interface_names = {"ana.1"};
  inteface_value_gpio2.values = {30.0};

  command_ptr->joint_names = {"gpio1", "gpio2"};
  command_ptr->interface_values = {inteface_value_gpio1, inteface_value_gpio2};
  controller.rt_command_ptr_.writeFromNonRT(command_ptr);

  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 30.0);
}

TEST_F(GpioCommandControllerTestSuite, CommandSuccessTestWithOnlyOneGpio)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto command_ptr = std::make_shared<CmdType>();
  control_msgs::msg::InterfaceValue inteface_value_gpio1;
  inteface_value_gpio1.interface_names = {"dig.1", "dig.2"};
  inteface_value_gpio1.values = {0.0, 1.0};

  command_ptr->joint_names = {"gpio1"};
  command_ptr->interface_values = {inteface_value_gpio1};
  controller.rt_command_ptr_.writeFromNonRT(command_ptr);

  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 3.1);
}

TEST_F(GpioCommandControllerTestSuite, NoCommandCheckTest)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 3.1);
}

TEST_F(GpioCommandControllerTestSuite, CommandCallbackTest)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 3.1);

  ASSERT_EQ(gpio_1_1_dig_state.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_state.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_state.get_value(), 3.1);

  auto node_state = controller.configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller.get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto command_pub = test_node.create_publisher<CmdType>(
    std::string(controller.get_node()->get_name()) + "/commands", rclcpp::SystemDefaultsQoS());
  CmdType command_msg;
  control_msgs::msg::InterfaceValue inteface_value_gpio1;
  inteface_value_gpio1.interface_names = {"dig.1", "dig.2"};
  inteface_value_gpio1.values = {0.0, 1.0};
  control_msgs::msg::InterfaceValue inteface_value_gpio2;
  inteface_value_gpio2.interface_names = {"ana.1"};
  inteface_value_gpio2.values = {30.0};

  command_msg.joint_names = {"gpio1", "gpio2"};
  command_msg.interface_values = {inteface_value_gpio1, inteface_value_gpio2};

  command_pub->publish(command_msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  const auto timeout = std::chrono::milliseconds{1};
  const auto until = controller.get_node()->get_clock()->now() + timeout;
  while (controller.get_node()->get_clock()->now() < until)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }

  ASSERT_EQ(
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 30.0);
}

TEST_F(GpioCommandControllerTestSuite, StateCallbackTest)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("gpios", std::vector<std::string>{"gpio1", "gpio2"});
  parameters.emplace_back(
    "command_interfaces.gpio1.ports", std::vector<std::string>{"dig.1", "dig.2"});
  parameters.emplace_back("command_interfaces.gpio2.ports", std::vector<std::string>{"ana.1"});
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  const auto result = controller.init("test_gpio_command_controller", "", 0, "", node_options);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd);
  command_ifs.emplace_back(gpio_1_2_dig_cmd);
  command_ifs.emplace_back(gpio_2_ana_cmd);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state);
  state_ifs.emplace_back(gpio_1_2_dig_state);
  state_ifs.emplace_back(gpio_2_ana_state);

  controller.assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_EQ(gpio_1_1_dig_cmd.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd.get_value(), 3.1);

  ASSERT_EQ(gpio_1_1_dig_state.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_state.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_state.get_value(), 3.1);

  auto node_state = controller.configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller.get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const StateType::SharedPtr) {};
  auto subscription = test_node.create_subscription<StateType>(
    std::string(controller.get_node()->get_name()) + "/gpio_states", 10, subs_callback);

  int max_sub_check_loop_count = 5;
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  while (max_sub_check_loop_count--)
  {
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
    {
      break;
    }
  }
  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                            "controller/broadcaster update loop";

  StateType gpio_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(gpio_state_msg, msg_info));

  ASSERT_EQ(gpio_state_msg.joint_names.size(), gpio_names.size());
  ASSERT_EQ(gpio_state_msg.joint_names[0], "gpio1");
  ASSERT_EQ(gpio_state_msg.joint_names[1], "gpio2");
  ASSERT_EQ(gpio_state_msg.interface_values[0].interface_names[0], "dig.1");
  ASSERT_EQ(gpio_state_msg.interface_values[0].interface_names[1], "dig.2");
  ASSERT_EQ(gpio_state_msg.interface_values[1].interface_names[0], "ana.1");
  ASSERT_EQ(gpio_state_msg.interface_values[0].values[0], 1.0);
  ASSERT_EQ(gpio_state_msg.interface_values[0].values[1], 0.0);
  ASSERT_EQ(gpio_state_msg.interface_values[1].values[0], 3.1);
}
