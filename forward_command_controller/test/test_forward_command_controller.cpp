// Copyright 2020 PAL Robotics SL.
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

#include "gmock/gmock.h"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "forward_command_controller/forward_command_controller.hpp"
#include "hardware_interface/joint_handle.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_forward_command_controller.hpp"

using CallbackReturn = forward_command_controller::ForwardCommandController::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using testing::SizeIs;
using testing::IsEmpty;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}

void ForwardCommandControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void ForwardCommandControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void ForwardCommandControllerTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendForwardCommandController>();
}

void ForwardCommandControllerTest::TearDown()
{
  controller_.reset(nullptr);
}

void ForwardCommandControllerTest::SetUpController()
{
  const auto result = controller_->init("forward_command_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_2_pos_cmd_);
  command_ifs.emplace_back(joint_3_pos_cmd_);
  controller_->assign_interfaces(std::move(command_ifs), {});
}

TEST_F(ForwardCommandControllerTest, JointsParameterNotSet)
{
  SetUpController();
  controller_->lifecycle_node_->declare_parameter("interface_name", "dummy");

  // configure failed, 'joints' paremeter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(ForwardCommandControllerTest, InterfaceParameterNotSet)
{
  SetUpController();

  // configure failed, 'interface_name' paremeter not set
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>()));
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter("interface_name", "");
}

TEST_F(ForwardCommandControllerTest, JointsParameterIsEmpty)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>()));
  controller_->lifecycle_node_->declare_parameter("interface_name", "");

  // configure failed, 'joints' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(ForwardCommandControllerTest, InterfaceParameterEmpty)
{
  SetUpController();

  // configure failed, 'interface_name' paremeter not set
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "");

  // configure failed, 'interface_name' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(ForwardCommandControllerTest, ConfigureParamsSuccess)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ForwardCommandControllerTest, ActivateWithWrongJointsNamesFails)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint4"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");

  // configure failed, 'joint4' not in interfaces
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  auto result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "joints",
      rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2"})));
  ASSERT_TRUE(result.successful);

  // configure failed, 'joint1' does not support 'acceleration_command' interface
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(ForwardCommandControllerTest, ActivateWithWrongInterfaceNameFails)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "acceleration");

  // configure failed, 'joint4' not in interfaces
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(ForwardCommandControllerTest, ActivateSuccess)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");

  // configure failed, 'joint4' not in interfaces
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ForwardCommandControllerTest, CommandSuccessTest)
{
  SetUpController();

  // configure controller
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful though no command has been send yet
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint commands are still the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 3.1);

  // send command
  auto command_ptr =
    std::make_shared<forward_command_controller::CmdType>();
  command_ptr->data = {10.0, 20.0, 30.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint commands have been modified
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 30.0);
}

TEST_F(ForwardCommandControllerTest, WrongCommandCheckTest)
{
  SetUpController();

  // configure controller
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // send command with wrong numnber of joints
  auto command_ptr =
    std::make_shared<forward_command_controller::CmdType>();
  command_ptr->data = {10.0, 20.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update failed, command size does not match number of joints
  ASSERT_EQ(controller_->update(), controller_interface::return_type::ERROR);

  // check joint commands are still the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 3.1);
}

TEST_F(ForwardCommandControllerTest, NoCommandCheckTest)
{
  SetUpController();

  // configure controller
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful, no command received yet
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint commands are still the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 3.1);
}

TEST_F(ForwardCommandControllerTest, CommandCallbackTest)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2", "joint3"}));
  controller_->lifecycle_node_->declare_parameter("interface_name", "position");

  // default values
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 3.1);

  auto node_state = controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller_->get_lifecycle_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // send a new command
  rclcpp::Node test_node("test_node");
  auto command_pub = test_node.create_publisher<std_msgs::msg::Float64MultiArray>(
    "commands", rclcpp::SystemDefaultsQoS());
  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data = {10.0, 20.0, 30.0};
  command_pub->publish(command_msg);

  // wait for command message to be passed
  ASSERT_EQ(wait_for(controller_->joints_command_subscriber_), rclcpp::WaitResultKind::Ready);

  // process callbacks
  rclcpp::spin_some(controller_->get_lifecycle_node()->get_node_base_interface());

  // update successful
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check command in handle was set
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 30.0);
}
