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
#include <vector>

#include "gtest/gtest.h"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/joint_handle.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_joint_group_position_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using CallbackReturn = forward_command_controller::ForwardCommandController::CallbackReturn;

void JointGroupPositionControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void JointGroupPositionControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void JointGroupPositionControllerTest::SetUp()
{
  // initialize robot
  test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
  test_robot_->init();

  // initialize controller
  controller_ = std::make_unique<FriendJointGroupPositionController>();
}

void JointGroupPositionControllerTest::TearDown()
{
  controller_.reset(nullptr);
}

void JointGroupPositionControllerTest::SetUpController()
{
  const auto result = controller_->init(test_robot_, "test_joint_group_position_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

void JointGroupPositionControllerTest::SetUpHandles()
{
  // get handles from test_robot_hardware
  joint1_pos_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint1",
    "position_command");
  joint2_pos_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint2",
    "position_command");
  joint3_pos_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint3",
    "position_command");

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_pos_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint2_pos_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint3_pos_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
}

TEST_F(JointGroupPositionControllerTest, ConfigureParamsTest)
{
  // joint handles not initialized yet
  ASSERT_TRUE(controller_->joint_cmd_handles_.empty());

  SetUpController();

  // configure failed, 'joints' paremeter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>()));

  // configure failed, 'joints' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  auto result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "joints",
      rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2"})));
  ASSERT_TRUE(result.successful);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handles initialized
  ASSERT_EQ(controller_->joint_cmd_handles_.size(), 2ul);
  ASSERT_EQ(controller_->joint_cmd_handles_[0].get_interface_name(), "position_command");
  ASSERT_EQ(controller_->joint_cmd_handles_[1].get_interface_name(), "position_command");
}

TEST_F(JointGroupPositionControllerTest, CheckParamsTest)
{
  // joint handles not initialized yet
  ASSERT_TRUE(controller_->joint_cmd_handles_.empty());

  SetUpController();

  // configure failed, interface name has already been set, with the wrong interface
  controller_->lifecycle_node_->declare_parameter("interface_name", "velocity_command");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  auto result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "interface_name",
      rclcpp::ParameterValue("position_command")));
  ASSERT_TRUE(result.successful);

  controller_->lifecycle_node_->declare_parameter(
    "joints",
    rclcpp::ParameterValue(std::vector<std::string>{"joint1", "joint2"}));

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handles initialized
  ASSERT_EQ(controller_->joint_cmd_handles_.size(), 2ul);
  ASSERT_EQ(controller_->joint_cmd_handles_[0].get_interface_name(), "position_command");
  ASSERT_EQ(controller_->joint_cmd_handles_[1].get_interface_name(), "position_command");
}
