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

void GripperControllerTest::SetUpTestCase() {}

void GripperControllerTest::TearDownTestCase() {}

void GripperControllerTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendGripperController>();
}

void GripperControllerTest::TearDown()
{
  try
  {
    controller_->get_node()->shutdown();
  }
  catch (...)
  {
    // ignore case where node is not initialized
  }
  controller_.reset(nullptr);  // this calls the dtor, but does not call shutdown transition
}

void GripperControllerTest::SetUpController(
  const std::string & controller_name = "test_gripper_action_position_controller",
  controller_interface::return_type expected_result = controller_interface::return_type::OK)
{
  const auto result =
    controller_->init(controller_name, "", 0, "", controller_->define_custom_node_options());
  ASSERT_EQ(result, expected_result);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_);
  state_ifs.emplace_back(this->joint_1_vel_state_);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(GripperControllerTest, ParametersNotSet)
{
  this->SetUpController(
    "test_gripper_action_position_controller_no_parameters",
    controller_interface::return_type::ERROR);

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, JointParameterIsEmpty)
{
  this->SetUpController(
    "test_gripper_action_position_controller_empty_joint",
    controller_interface::return_type::ERROR);

  // configure failed, 'joints' is empty
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(GripperControllerTest, ConfigureParamsSuccess)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "joint1"});

  rclcpp::spin_some(this->controller_->get_node()->get_node_base_interface());

  // configure successful
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(1lu));
  ASSERT_THAT(
    cmd_if_conf.names,
    UnorderedElementsAre(std::string("joint1/") + hardware_interface::HW_IF_POSITION));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto state_if_conf = this->controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2lu));
  ASSERT_THAT(state_if_conf.names, UnorderedElementsAre("joint1/position", "joint1/velocity"));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
