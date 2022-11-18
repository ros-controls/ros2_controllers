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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "test_gripper_controllers.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using GripperCommandAction = control_msgs::action::GripperCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;

void GripperControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void GripperControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void GripperControllerTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendGripperController>();
}

void GripperControllerTest::TearDown() { controller_.reset(nullptr); }

void GripperControllerTest::SetUpController()
{
  const auto result = controller_->init("gripper_controller");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_pos_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(GripperControllerTest, ParametersNotSet)
{
  SetUpController();

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, JointParameterIsEmpty)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", ""});

  // configure failed, 'joints' is empty
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, ConfigureParamsSuccess)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint_1"});

  // configure successful
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(GripperControllerTest, ActivateWithWrongJointsNamesFails)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "unicorn_joint"});

  // activate failed, 'joint4' is not a valid joint name for the hardware
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, ActivateSuccess)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint1"});

  // activate successful
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(GripperControllerTest, ActivateDeactivateActivateSuccess)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint1"});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // re-assign interfaces
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_pos_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
