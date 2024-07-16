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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "test_gripper_controllers.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using GripperCommandAction = control_msgs::action::GripperCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
using testing::SizeIs;
using testing::UnorderedElementsAre;

template <typename T>
void GripperControllerTest<T>::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

template <typename T>
void GripperControllerTest<T>::TearDownTestCase()
{
  rclcpp::shutdown();
}

template <typename T>
void GripperControllerTest<T>::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendGripperController<T::value>>();
}

template <typename T>
void GripperControllerTest<T>::TearDown()
{
  controller_.reset(nullptr);
}

template <typename T>
void GripperControllerTest<T>::SetUpController()
{
  const auto result = controller_->init("gripper_controller");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_);
  state_ifs.emplace_back(this->joint_1_vel_state_);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

using TestTypes = ::testing::Types<
  std::integral_constant<const char *, HW_IF_POSITION>,
  std::integral_constant<const char *, HW_IF_EFFORT>>;
TYPED_TEST_SUITE(GripperControllerTest, TestTypes);

TYPED_TEST(GripperControllerTest, ParametersNotSet)
{
  this->SetUpController();

  // configure failed, 'joints' parameter not set
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TYPED_TEST(GripperControllerTest, JointParameterIsEmpty)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", ""});

  // configure failed, 'joints' is empty
  ASSERT_EQ(
    this->controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TYPED_TEST(GripperControllerTest, ConfigureParamsSuccess)
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
  ASSERT_THAT(cmd_if_conf.names, UnorderedElementsAre(std::string("joint_1/") + TypeParam::value));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto state_if_conf = this->controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2lu));
  ASSERT_THAT(state_if_conf.names, UnorderedElementsAre("joint_1/position", "joint_1/velocity"));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TYPED_TEST(GripperControllerTest, ActivateWithWrongJointsNamesFails)
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

TYPED_TEST(GripperControllerTest, ActivateSuccess)
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

TYPED_TEST(GripperControllerTest, ActivateDeactivateActivateSuccess)
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
