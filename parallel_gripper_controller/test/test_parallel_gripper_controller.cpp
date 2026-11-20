// Copyright 2022 ros2_control development team
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

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_parallel_gripper_controller.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using GripperCommandAction = control_msgs::action::ParallelGripperCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommandAction>;
using testing::SizeIs;
using testing::UnorderedElementsAre;

namespace
{
class ActionHarness
{
public:
  explicit ActionHarness(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & controller)
  {
    client_node = std::make_shared<rclcpp::Node>("parallel_gripper_test_client");
    executor.add_node(controller);
    executor.add_node(client_node);
    client = rclcpp_action::create_client<GripperCommandAction>(
      client_node, "/test_gripper_action_position_controller/gripper_cmd");
  }

  ClientGoalHandle::SharedPtr send(double position)
  {
    GripperCommandAction::Goal goal;
    goal.command.position = {position};
    return send(goal);
  }

  ClientGoalHandle::SharedPtr send(const GripperCommandAction::Goal & goal)
  {
    if (!client->wait_for_action_server(std::chrono::milliseconds(500)))
    {
      throw std::runtime_error("action server unavailable");
    }
    auto future = client->async_send_goal(goal);
    if (
      executor.spin_until_future_complete(future, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("goal response timed out");
    }
    return future.get();
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<GripperCommandAction>::SharedPtr client;
};
}  // namespace

void GripperControllerTest::SetUpTestCase() {}

void GripperControllerTest::TearDownTestCase() {}

void GripperControllerTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendGripperController>();
  joint_1_pos_state_ = std::make_shared<hardware_interface::StateInterface>(
    joint_name_, hardware_interface::HW_IF_POSITION);
  std::ignore = joint_1_pos_state_->set_value(joint_states_[0]);
  joint_1_vel_state_ = std::make_shared<hardware_interface::StateInterface>(
    joint_name_, hardware_interface::HW_IF_VELOCITY);
  std::ignore = joint_1_vel_state_->set_value(joint_states_[1]);
  joint_1_cmd_ = std::make_shared<hardware_interface::CommandInterface>(
    joint_name_, hardware_interface::HW_IF_POSITION);
  std::ignore = joint_1_cmd_->set_value(joint_commands_[0]);
  joint_1_effort_cmd_ = std::make_shared<hardware_interface::CommandInterface>(
    joint_name_, hardware_interface::HW_IF_EFFORT);
  std::ignore = joint_1_effort_cmd_->set_value(joint_effort_commands_[0]);
  joint_1_speed_cmd_ = std::make_shared<hardware_interface::CommandInterface>(
    joint_name_, hardware_interface::HW_IF_VELOCITY);
  std::ignore = joint_1_speed_cmd_->set_value(joint_speed_commands_[0]);
}

void GripperControllerTest::TearDown() { controller_.reset(nullptr); }

void GripperControllerTest::SetUpController(
  const std::string & controller_name = "test_gripper_action_position_controller",
  controller_interface::return_type expected_result = controller_interface::return_type::OK)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = controller_name;
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = controller_->define_custom_node_options();

  const auto result = controller_->init(params);
  ASSERT_EQ(result, expected_result);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(GripperControllerTest, ParametersNotSet)
{
  this->SetUpController(
    "test_gripper_action_position_controller_no_parameters",
    controller_interface::return_type::ERROR);
}

TEST_F(GripperControllerTest, JointParameterIsEmpty)
{
  this->SetUpController(
    "test_gripper_action_position_controller_empty_joint",
    controller_interface::return_type::ERROR);
}

TEST_F(GripperControllerTest, ConfigureParamsSuccess)
{
  this->SetUpController();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  executor.spin_some();

  ASSERT_TRUE(configure_succeeds(controller_));

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

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
}

TEST_F(GripperControllerTest, ActivateDeactivateActivateSuccess)
{
  this->SetUpController();

  this->controller_->get_node()->set_parameter({"joint", "joint1"});

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  this->controller_->release_interfaces();

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
}

TEST_F(GripperControllerTest, ActivateWithEffortInterfaceSuccess)
{
  this->SetUpController("test_gripper_controller_with_effort");

  this->controller_->release_interfaces();
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_effort_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_TRUE(configure_succeeds(controller_));

  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, testing::Contains("joint1/effort"));

  ASSERT_TRUE(activate_succeeds(controller_));
}

TEST_F(GripperControllerTest, ActivateWithVelocityInterfaceSuccess)
{
  this->SetUpController("test_gripper_controller_with_velocity");

  this->controller_->release_interfaces();
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_speed_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_TRUE(configure_succeeds(controller_));

  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, testing::Contains("joint1/velocity"));

  ASSERT_TRUE(activate_succeeds(controller_));
}

TEST_F(GripperControllerTest, ActivateWithEffortAndVelocityInterfaceSuccess)
{
  this->SetUpController("test_gripper_controller_with_effort_and_velocity");

  this->controller_->release_interfaces();
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_effort_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_speed_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_TRUE(configure_succeeds(controller_));

  auto cmd_if_conf = this->controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, testing::Contains("joint1/effort"));
  ASSERT_THAT(cmd_if_conf.names, testing::Contains("joint1/velocity"));

  ASSERT_TRUE(activate_succeeds(controller_));
}

TEST_F(GripperControllerTest, DeactivateWithEffortAndVelocitySuccess)
{
  this->SetUpController("test_gripper_controller_with_effort_and_velocity");

  this->controller_->release_interfaces();
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(this->joint_1_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_effort_cmd_, nullptr);
  command_ifs.emplace_back(this->joint_1_speed_cmd_, nullptr);
  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(this->joint_1_pos_state_, nullptr);
  state_ifs.emplace_back(this->joint_1_vel_state_, nullptr);
  this->controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));
}

TEST_F(GripperControllerTest, ActivateFailsWhenEffortInterfaceMissing)
{
  // effort interface configured in params but NOT provided as a hardware interface
  this->SetUpController("test_gripper_controller_with_effort");

  ASSERT_TRUE(configure_succeeds(controller_));

  // on_activate should FAIL because effort configured but not found
  ASSERT_EQ(
    this->controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}

TEST_F(GripperControllerTest, ActionCancelControl)
{
  this->SetUpController();
  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto goal_handle = action.send(2.0);
  ASSERT_TRUE(goal_handle);
  auto result = action.client->async_get_result(goal_handle);
  EXPECT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::milliseconds(100)),
    rclcpp::FutureReturnCode::TIMEOUT);

  auto cancel = action.client->async_cancel_goal(goal_handle);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(cancel, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(GripperControllerTest, DeactivateTerminatesExecutingGoal)
{
  this->SetUpController();
  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto goal_handle = action.send(2.0);
  ASSERT_TRUE(goal_handle);
  auto result = action.client->async_get_result(goal_handle);
  EXPECT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::milliseconds(100)),
    rclcpp::FutureReturnCode::TIMEOUT);

  ASSERT_TRUE(deactivate_succeeds(controller_));
  const auto status =
    action.executor.spin_until_future_complete(result, std::chrono::seconds(1));
  EXPECT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
  if (status == rclcpp::FutureReturnCode::SUCCESS)
  {
    EXPECT_TRUE(
      result.get().code == rclcpp_action::ResultCode::CANCELED ||
      result.get().code == rclcpp_action::ResultCode::ABORTED);
  }
}

TEST_F(GripperControllerTest, InactiveRejectsNewGoal)
{
  this->SetUpController();
  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  ASSERT_TRUE(action.client->wait_for_action_server(std::chrono::milliseconds(500)));
  ASSERT_TRUE(deactivate_succeeds(controller_));

  const bool available =
    action.client->wait_for_action_server(std::chrono::milliseconds(200));
  if (available)
  {
    GripperCommandAction::Goal goal;
    goal.command.position = {2.0};
    auto response = action.client->async_send_goal(goal);
    const auto status =
      action.executor.spin_until_future_complete(response, std::chrono::milliseconds(500));
    EXPECT_NE(status, rclcpp::FutureReturnCode::INTERRUPTED);
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
      EXPECT_FALSE(response.get());
    }
  }
}

TEST_F(GripperControllerTest, PreemptTerminatesOldGoal)
{
  this->SetUpController();
  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto old_goal = action.send(2.0);
  ASSERT_TRUE(old_goal);
  auto old_result = action.client->async_get_result(old_goal);
  EXPECT_EQ(
    action.executor.spin_until_future_complete(old_result, std::chrono::milliseconds(100)),
    rclcpp::FutureReturnCode::TIMEOUT);

  auto new_goal = action.send(3.0);
  ASSERT_TRUE(new_goal);
  auto new_result = action.client->async_get_result(new_goal);
  const auto old_status =
    action.executor.spin_until_future_complete(old_result, std::chrono::seconds(1));
  EXPECT_EQ(old_status, rclcpp::FutureReturnCode::SUCCESS);
  if (old_status == rclcpp::FutureReturnCode::SUCCESS)
  {
    EXPECT_TRUE(
      old_result.get().code == rclcpp_action::ResultCode::CANCELED ||
      old_result.get().code == rclcpp_action::ResultCode::ABORTED);
  }

  auto cancel = action.client->async_cancel_goal(new_goal);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(cancel, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(new_result, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
}

TEST_F(GripperControllerTest, ImmediateSuccessControl)
{
  this->SetUpController();
  this->controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto goal_handle = action.send(joint_states_[0]);
  ASSERT_TRUE(goal_handle);
  auto result = action.client->async_get_result(goal_handle);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.get().result->reached_goal);
  EXPECT_FALSE(result.get().result->stalled);
  ASSERT_THAT(result.get().result->state.position, SizeIs(1lu));
  EXPECT_DOUBLE_EQ(result.get().result->state.position[0], joint_states_[0]);
  EXPECT_TRUE(result.get().result->state.effort.empty());
}

TEST_F(GripperControllerTest, StallResultDoesNotPublishUnknownEffort)
{
  this->SetUpController();
  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter({"stall_velocity_threshold", 3.0});
  controller_->get_node()->set_parameter({"stall_timeout", 0.0});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto goal_handle = action.send(2.0);
  ASSERT_TRUE(goal_handle);
  auto result = action.client->async_get_result(goal_handle);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(result.get().result->stalled);
  EXPECT_TRUE(result.get().result->state.effort.empty());
}

TEST_F(GripperControllerTest, RejectsGoalForAnotherJoint)
{
  this->SetUpController();
  controller_->get_node()->set_parameter({"joint", "joint1"});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  GripperCommandAction::Goal goal;
  goal.command.name = {"other_joint"};
  goal.command.position = {2.0};
  EXPECT_FALSE(action.send(goal));
}

TEST_F(GripperControllerTest, ZeroToleranceAcceptsExactGoal)
{
  this->SetUpController();
  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter({"goal_tolerance", 0.0});
  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ActionHarness action(controller_->get_node()->get_node_base_interface());
  auto goal_handle = action.send(joint_states_[0]);
  ASSERT_TRUE(goal_handle);
  auto result = action.client->async_get_result(goal_handle);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    action.executor.spin_until_future_complete(result, std::chrono::seconds(1)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
