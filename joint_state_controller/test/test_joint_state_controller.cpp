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
#include "joint_state_controller/joint_state_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_joint_state_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using std::placeholders::_1;

namespace
{
constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}  // namespace

void JointStateControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void JointStateControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void JointStateControllerTest::SetUp()
{
  // initialize robot
  test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
  test_robot_->init();

  // initialize controller
  state_controller_ = std::make_unique<FriendJointStateController>();
}

void JointStateControllerTest::TearDown()
{
  state_controller_.reset(nullptr);
}

void JointStateControllerTest::SetUpStateController()
{
  const auto result = state_controller_->init(test_robot_, "joint_state_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

TEST_F(JointStateControllerTest, ConfigureErrorTest)
{
  // joint state not initialized yet
  ASSERT_TRUE(state_controller_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.effort.empty());

  // dynamic joint state not initialized yet
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers not initialized yet
  ASSERT_FALSE(state_controller_->joint_state_publisher_);
  ASSERT_FALSE(state_controller_->dynamic_joint_state_publisher_);

  // configure failed
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  // check state remains unchanged

  // joint state still not initialized
  ASSERT_TRUE(state_controller_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.effort.empty());

  // dynamic joint state still not initialized
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers still not initialized
  ASSERT_FALSE(state_controller_->joint_state_publisher_);
  ASSERT_FALSE(state_controller_->dynamic_joint_state_publisher_);
}

TEST_F(JointStateControllerTest, ConfigureSuccessTest)
{
  // joint state not initialized yet
  ASSERT_TRUE(state_controller_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_controller_->joint_state_msg_.effort.empty());

  // dynamic joint state not initialized yet
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_controller_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers not initialized yet
  ASSERT_FALSE(state_controller_->joint_state_publisher_);
  ASSERT_FALSE(state_controller_->dynamic_joint_state_publisher_);

  SetUpStateController();
  // configure ok
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const std::vector<std::string> JOINT_NAMES = {"joint1", "joint2", "joint3"};
  const size_t NUM_JOINTS = JOINT_NAMES.size();
  const std::vector<std::string> IF_NAMES = {"position", "velocity", "effort"};
  const size_t NUM_IFS = IF_NAMES.size();

  // joint state initialized
  ASSERT_EQ(state_controller_->joint_state_msg_.name.size(), NUM_JOINTS);
  ASSERT_EQ(state_controller_->joint_state_msg_.position.size(), NUM_JOINTS);
  ASSERT_EQ(state_controller_->joint_state_msg_.velocity.size(), NUM_JOINTS);
  ASSERT_EQ(state_controller_->joint_state_msg_.effort.size(), NUM_JOINTS);
  ASSERT_EQ(state_controller_->joint_state_msg_.name, JOINT_NAMES);

  // dynamic joint state initialized
  ASSERT_EQ(state_controller_->dynamic_joint_state_msg_.joint_names.size(), NUM_JOINTS);
  ASSERT_EQ(state_controller_->dynamic_joint_state_msg_.interface_values.size(), NUM_IFS);
  ASSERT_EQ(state_controller_->dynamic_joint_state_msg_.joint_names, JOINT_NAMES);
  ASSERT_EQ(
    state_controller_->dynamic_joint_state_msg_.interface_values[0].interface_names,
    IF_NAMES);
  ASSERT_EQ(
    state_controller_->dynamic_joint_state_msg_.interface_values[1].interface_names,
    IF_NAMES);
  ASSERT_EQ(
    state_controller_->dynamic_joint_state_msg_.interface_values[2].interface_names,
    IF_NAMES);

  // publishers initialized
  ASSERT_TRUE(state_controller_->joint_state_publisher_);
  ASSERT_TRUE(state_controller_->dynamic_joint_state_publisher_);
}

TEST_F(JointStateControllerTest, UpdateTest)
{
  SetUpStateController();

  auto node_state = state_controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // publishers not activated yet
  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::ERROR);

  node_state = state_controller_->get_lifecycle_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);
}

TEST_F(JointStateControllerTest, JointStatePublishTest)
{
  SetUpStateController();

  ASSERT_EQ(test_robot_->write(), hardware_interface::return_type::OK);

  auto node_state = state_controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_controller_->get_lifecycle_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const sensor_msgs::msg::JointState::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10,
    subs_callback);

  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  sensor_msgs::msg::JointState joint_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(joint_state_msg, msg_info));

  // checking positions is enough
  const size_t NUM_JOINTS = 3;
  ASSERT_EQ(joint_state_msg.position.size(), NUM_JOINTS);
  ASSERT_EQ(joint_state_msg.position[0], 1.1);
  ASSERT_EQ(joint_state_msg.position[1], 2.2);
  ASSERT_EQ(joint_state_msg.position[2], 3.3);
}

TEST_F(JointStateControllerTest, DynamicJointStatePublishTest)
{
  SetUpStateController();

  ASSERT_EQ(test_robot_->write(), hardware_interface::return_type::OK);

  auto node_state = state_controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_controller_->get_lifecycle_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const control_msgs::msg::DynamicJointState::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<control_msgs::msg::DynamicJointState>(
    "dynamic_joint_states",
    10,
    subs_callback);

  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  control_msgs::msg::DynamicJointState dynamic_joint_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(dynamic_joint_state_msg, msg_info));

  // checking positions is enough
  const size_t NUM_JOINTS = 3;
  ASSERT_EQ(dynamic_joint_state_msg.joint_names.size(), NUM_JOINTS);
  ASSERT_EQ(dynamic_joint_state_msg.interface_values[0].values[0], 1.1);
  ASSERT_EQ(dynamic_joint_state_msg.interface_values[1].values[0], 2.2);
  ASSERT_EQ(dynamic_joint_state_msg.interface_values[2].values[0], 3.3);
}
