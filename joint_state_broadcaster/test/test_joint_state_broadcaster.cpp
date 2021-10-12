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

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_state_broadcaster/joint_state_broadcaster.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_joint_state_broadcaster.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedStateInterface;
using std::placeholders::_1;
using testing::Each;
using testing::ElementsAreArray;
using testing::IsEmpty;
using testing::SizeIs;

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

void JointStateBroadcasterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void JointStateBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

void JointStateBroadcasterTest::SetUp()
{
  // initialize broadcaster
  state_broadcaster_ = std::make_unique<FriendJointStateBroadcaster>();
}

void JointStateBroadcasterTest::TearDown() { state_broadcaster_.reset(nullptr); }

void JointStateBroadcasterTest::SetUpStateBroadcaster()
{
  const auto result = state_broadcaster_->init("joint_state_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_2_pos_state_);
  state_ifs.emplace_back(joint_3_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  state_ifs.emplace_back(joint_2_vel_state_);
  state_ifs.emplace_back(joint_3_vel_state_);
  state_ifs.emplace_back(joint_1_eff_state_);
  state_ifs.emplace_back(joint_2_eff_state_);
  state_ifs.emplace_back(joint_3_eff_state_);

  state_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

TEST_F(JointStateBroadcasterTest, ConfigureErrorTest)
{
  // joint state not initialized yet
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.effort.empty());

  // dynamic joint state not initialized yet
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers not initialized yet
  ASSERT_FALSE(state_broadcaster_->joint_state_publisher_);
  ASSERT_FALSE(state_broadcaster_->dynamic_joint_state_publisher_);

  // configure failed
  ASSERT_THROW(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), std::exception);

  SetUpStateBroadcaster();
  // check state remains unchanged

  // joint state still not initialized
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.effort.empty());

  // dynamic joint state still not initialized
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers still not initialized
  ASSERT_FALSE(state_broadcaster_->joint_state_publisher_);
  ASSERT_FALSE(state_broadcaster_->dynamic_joint_state_publisher_);
}

TEST_F(JointStateBroadcasterTest, ConfigureSuccessTest)
{
  // joint state not initialized yet
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.name, IsEmpty());
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.position, IsEmpty());
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.velocity, IsEmpty());
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.effort, IsEmpty());

  // dynamic joint state not initialized yet
  ASSERT_THAT(state_broadcaster_->dynamic_joint_state_msg_.joint_names, IsEmpty());
  ASSERT_THAT(state_broadcaster_->dynamic_joint_state_msg_.interface_values, IsEmpty());

  // publishers not initialized yet
  ASSERT_FALSE(state_broadcaster_->joint_state_publisher_);
  ASSERT_FALSE(state_broadcaster_->dynamic_joint_state_publisher_);

  SetUpStateBroadcaster();
  // configure ok
  ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const size_t NUM_JOINTS = joint_names_.size();
  const std::vector<std::string> IF_NAMES = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  const size_t NUM_IFS = IF_NAMES.size();

  // joint state initialized
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.name, ElementsAreArray(joint_names_));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.position, SizeIs(NUM_JOINTS));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.velocity, SizeIs(NUM_JOINTS));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.effort, SizeIs(NUM_JOINTS));

  // dynamic joint state initialized
  ASSERT_THAT(state_broadcaster_->dynamic_joint_state_msg_.joint_names, SizeIs(NUM_JOINTS));
  ASSERT_THAT(state_broadcaster_->dynamic_joint_state_msg_.interface_values, SizeIs(NUM_IFS));
  ASSERT_THAT(
    state_broadcaster_->dynamic_joint_state_msg_.joint_names, ElementsAreArray(joint_names_));
  ASSERT_THAT(
    state_broadcaster_->dynamic_joint_state_msg_.interface_values[0].interface_names,
    ElementsAreArray(IF_NAMES));
  ASSERT_THAT(
    state_broadcaster_->dynamic_joint_state_msg_.interface_values[1].interface_names,
    ElementsAreArray(IF_NAMES));
  ASSERT_THAT(
    state_broadcaster_->dynamic_joint_state_msg_.interface_values[2].interface_names,
    ElementsAreArray(IF_NAMES));

  // publishers initialized
  ASSERT_TRUE(state_broadcaster_->joint_state_publisher_);
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_publisher_);
}

TEST_F(JointStateBroadcasterTest, UpdateTest)
{
  SetUpStateBroadcaster();

  auto node_state = state_broadcaster_->configure();
  node_state = state_broadcaster_->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(
    state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

void JointStateBroadcasterTest::test_published_joint_state_message(const std::string & topic)
{
  auto node_state = state_broadcaster_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_broadcaster_->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const sensor_msgs::msg::JointState::SharedPtr) {};
  auto subscription =
    test_node.create_subscription<sensor_msgs::msg::JointState>(topic, 10, subs_callback);

  ASSERT_EQ(
    state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  sensor_msgs::msg::JointState joint_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(joint_state_msg, msg_info));

  const size_t NUM_JOINTS = joint_names_.size();
  ASSERT_THAT(joint_state_msg.name, SizeIs(NUM_JOINTS));
  // the order in the message may be different
  // we only check that all values in this test are present in the message
  // and that it is the same across the interfaces
  // for test purposes they are mapped to the same doubles
  ASSERT_THAT(joint_state_msg.position, ElementsAreArray(joint_values_));
  ASSERT_THAT(joint_state_msg.velocity, ElementsAreArray(joint_state_msg.position));
  ASSERT_THAT(joint_state_msg.effort, ElementsAreArray(joint_state_msg.position));
}

TEST_F(JointStateBroadcasterTest, JointStatePublishTest)
{
  SetUpStateBroadcaster();

  test_published_joint_state_message("joint_states");
}

TEST_F(JointStateBroadcasterTest, JointStatePublishTestLocalTopic)
{
  SetUpStateBroadcaster();
  state_broadcaster_->get_node()->set_parameter({"use_local_topics", true});

  test_published_joint_state_message("joint_state_broadcaster/joint_states");
}

void JointStateBroadcasterTest::test_published_dynamic_joint_state_message(
  const std::string & topic)
{
  auto node_state = state_broadcaster_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_broadcaster_->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const control_msgs::msg::DynamicJointState::SharedPtr) {};
  auto subscription =
    test_node.create_subscription<control_msgs::msg::DynamicJointState>(topic, 10, subs_callback);

  ASSERT_EQ(
    state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  control_msgs::msg::DynamicJointState dynamic_joint_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(dynamic_joint_state_msg, msg_info));

  const size_t NUM_JOINTS = 3;
  const auto INTERFACE_NAMES = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  ASSERT_THAT(dynamic_joint_state_msg.joint_names, SizeIs(NUM_JOINTS));
  // the order in the message may be different
  // we only check that all values in this test are present in the message
  // and that it is the same across the interfaces
  // for test purposes they are mapped to the same doubles
  for (size_t i = 0; i < dynamic_joint_state_msg.joint_names.size(); ++i)
  {
    ASSERT_THAT(
      dynamic_joint_state_msg.interface_values[i].interface_names,
      ElementsAreArray(INTERFACE_NAMES));
    const auto index_in_original_order = std::distance(
      joint_names_.cbegin(),
      std::find(
        joint_names_.cbegin(), joint_names_.cend(), dynamic_joint_state_msg.joint_names[i]));
    ASSERT_THAT(
      dynamic_joint_state_msg.interface_values[i].values,
      Each(joint_values_[index_in_original_order]));
  }
}

TEST_F(JointStateBroadcasterTest, DynamicJointStatePublishTest)
{
  SetUpStateBroadcaster();

  test_published_dynamic_joint_state_message("dynamic_joint_states");
}

TEST_F(JointStateBroadcasterTest, DynamicJointStatePublishTestLocalTopic)
{
  SetUpStateBroadcaster();
  state_broadcaster_->get_node()->set_parameter({"use_local_topics", true});

  test_published_dynamic_joint_state_message("joint_state_broadcaster/dynamic_joint_states");
}

TEST_F(JointStateBroadcasterTest, ExtraJointStatePublishTest)
{
  // joint state not initialized yet
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.name.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.position.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.velocity.empty());
  ASSERT_TRUE(state_broadcaster_->joint_state_msg_.effort.empty());

  // dynamic joint state not initialized yet
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.joint_names.empty());
  ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_msg_.interface_values.empty());

  // publishers not initialized yet
  ASSERT_FALSE(state_broadcaster_->joint_state_publisher_);
  ASSERT_FALSE(state_broadcaster_->dynamic_joint_state_publisher_);

  SetUpStateBroadcaster();

  // Add extra joints as parameters
  const std::vector<std::string> extra_joint_names = {"extra1", "extra2", "extra3"};
  state_broadcaster_->get_node()->set_parameter({"extra_joints", extra_joint_names});

  // configure ok
  ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  std::vector<std::string> all_joint_names = joint_names_;
  all_joint_names.insert(all_joint_names.end(), extra_joint_names.begin(), extra_joint_names.end());
  const size_t NUM_JOINTS = all_joint_names.size();
  const std::vector<std::string> IF_NAMES = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};

  // joint state initialized
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.name, ElementsAreArray(all_joint_names));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.position, SizeIs(NUM_JOINTS));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.velocity, SizeIs(NUM_JOINTS));
  ASSERT_THAT(state_broadcaster_->joint_state_msg_.effort, SizeIs(NUM_JOINTS));

  // dynamic joint state initialized
  ASSERT_THAT(state_broadcaster_->dynamic_joint_state_msg_.joint_names, SizeIs(NUM_JOINTS));
}
