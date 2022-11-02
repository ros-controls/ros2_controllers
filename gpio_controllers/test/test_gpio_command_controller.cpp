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

#include <stddef.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_gpio_command_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using CmdType = std_msgs::msg::Float64MultiArray;
using StateType = control_msgs::msg::DynamicJointState;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}  // namespace

void GpioCommandControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void GpioCommandControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void GpioCommandControllerTest::SetUp()
{
  controller_ = std::make_unique<FriendGpioCommandController>();
}

void GpioCommandControllerTest::TearDown() { controller_.reset(nullptr); }

void GpioCommandControllerTest::SetUpController()
{
  const auto result = controller_->init("test_gpio_command_controller");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(gpio_1_1_dig_cmd_);
  command_ifs.emplace_back(gpio_1_2_dig_cmd_);
  command_ifs.emplace_back(gpio_2_ana_cmd_);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(gpio_1_1_dig_state_);
  state_ifs.emplace_back(gpio_1_2_dig_state_);
  state_ifs.emplace_back(gpio_2_ana_state_);
  
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(GpioCommandControllerTest, GpiosParameterNotSet)
{
  SetUpController();

  // configure failed, 'gpios' parameter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(GpioCommandControllerTest, GpiosParameterIsEmpty)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", std::vector<std::string>()});

  // configure failed, 'gpios' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(GpioCommandControllerTest, GpioWithMissingGpioParams)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", std::vector<std::string>{"gpio1", "gpio2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>()});
  // // activate failed, command interface for 'gpio2' is not set up
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(GpioCommandControllerTest, ConfigureAndActivateParamsSuccess)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(GpioCommandControllerTest, ActivateWithWrongGpiosNamesFails)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", std::vector<std::string>{"gpio1", "gpio4"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio4", std::vector<std::string>{"ana.1"}});
  // // activate failed, 'gpio4' is not a valid gpio name for the hardware
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(GpioCommandControllerTest, CommandSuccessTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});


  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful though no command has been send yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check gpio commands are still the default ones
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 3.1);

  // send command
  auto command_ptr = std::make_shared<CmdType>();
  command_ptr->data = {0.0, 1.0, 30.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check gpio commands have been modified
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 30.0);
}

TEST_F(GpioCommandControllerTest, WrongCommandCheckTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});


  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);


  // send command with wrong number of gpios
  auto command_ptr = std::make_shared<CmdType>();
  command_ptr->data = {0.0, 20.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update failed, command size does not match number of gpios
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);

  // check gpio commands are still the default ones
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 3.1);
}

TEST_F(GpioCommandControllerTest, NoCommandCheckTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});


  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check gpio commands are still the default ones
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 3.1);
}

TEST_F(GpioCommandControllerTest, CommandCallbackTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});
  controller_->get_node()->set_parameter(
    {"state_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"state_interfaces.gpio2", std::vector<std::string>{"ana.1"}});


  // default values
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 3.1);

  ASSERT_EQ(gpio_1_1_dig_state_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_state_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_state_.get_value(), 3.1);

  auto node_state = controller_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // send a new command
  rclcpp::Node test_node("test_node");
  auto command_pub = test_node.create_publisher<CmdType>(
    std::string(controller_->get_node()->get_name()) + "/commands", rclcpp::SystemDefaultsQoS());
  CmdType command_msg;
  command_msg.data = {0.0, 1.0, 30.0};
  command_pub->publish(command_msg);

  // wait for command message to be passed
  ASSERT_EQ(wait_for(controller_->gpios_command_subscriber_), rclcpp::WaitResultKind::Ready);

  // process callbacks
  rclcpp::spin_some(controller_->get_node()->get_node_base_interface());

  // update successful
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 30.0);
}

TEST_F(GpioCommandControllerTest, StateCallbackTest)
{
  SetUpController();
  controller_->get_node()->set_parameter({"gpios", gpio_names_});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"command_interfaces.gpio2", std::vector<std::string>{"ana.1"}});
  controller_->get_node()->set_parameter(
    {"state_interfaces.gpio1", std::vector<std::string>{"dig.1", "dig.2"}});
  controller_->get_node()->set_parameter(
    {"state_interfaces.gpio2", std::vector<std::string>{"ana.1"}});


  // default values
  ASSERT_EQ(gpio_1_1_dig_cmd_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_cmd_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_cmd_.get_value(), 3.1);

  ASSERT_EQ(gpio_1_1_dig_state_.get_value(), 1.0);
  ASSERT_EQ(gpio_1_2_dig_state_.get_value(), 0.0);
  ASSERT_EQ(gpio_2_ana_state_.get_value(), 3.1);

  auto node_state = controller_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // send a new command
  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const StateType::SharedPtr) {};
  auto subscription = test_node.create_subscription<StateType>(
    std::string(controller_->get_node()->get_name()) + "/gpio_states", 10, subs_callback);

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  rclcpp::WaitSet wait_set;          // block used to wait on message
  wait_set.add_subscription(subscription);
  while (max_sub_check_loop_count--)
  {
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    // check if message has been received
    if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
    {
      break;
    }
  }
  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                            "controller/broadcaster update loop";

  // take message from subscription
  StateType gpio_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(gpio_state_msg, msg_info));

  ASSERT_EQ(gpio_state_msg.joint_names.size(), gpio_names_.size());
  ASSERT_EQ(gpio_state_msg.joint_names[0], "gpio1");
  ASSERT_EQ(gpio_state_msg.joint_names[1], "gpio2");
  ASSERT_EQ(gpio_state_msg.interface_values[0].interface_names[0], "dig.1");
  ASSERT_EQ(gpio_state_msg.interface_values[0].interface_names[1], "dig.2");
  ASSERT_EQ(gpio_state_msg.interface_values[1].interface_names[0], "ana.1");
  ASSERT_EQ(gpio_state_msg.interface_values[0].values[0], 1.0);
  ASSERT_EQ(gpio_state_msg.interface_values[0].values[1], 0.0);
  ASSERT_EQ(gpio_state_msg.interface_values[1].values[0], 3.1);
}
