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

#include "test_parallel_gripper_controller.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using GripperCommandAction = control_msgs::action::ParallelGripperCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
using testing::SizeIs;
using testing::UnorderedElementsAre;

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
  const auto result =
    controller_->init("gripper_controller", "", 0, "", controller_->define_custom_node_options());
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_);
  state_ifs.emplace_back(this->joint_1_vel_state_);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(GripperControllerTest, ParametersNotSet)
{
  this->SetUpController();

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, JointParameterIsEmpty)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", ""});

  // configure failed, 'joints' is empty
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, ConfigureParamsSuccess)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "joint_1"});

  // configure successful
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(1lu));
  ASSERT_THAT(
    cmd_if_conf.names,
    UnorderedElementsAre(std::string("joint_1/") + hardware_interface::HW_IF_POSITION));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto state_if_conf = this->controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2lu));
  ASSERT_THAT(state_if_conf.names, UnorderedElementsAre("joint_1/position", "joint_1/velocity"));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(GripperControllerTest, ActivateWithWrongJointsNamesFails)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "unicorn_joint"});

  // activate failed, 'joint4' is not a valid joint name for the hardware
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, ActivateSuccess)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "joint1"});

  // activate successful
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(GripperControllerTest, ActivateDeactivateActivateSuccess)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "joint1"});

  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    this->controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // re-assign interfaces
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_);
  state_ifs.emplace_back(this->joint_1_vel_state_);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
