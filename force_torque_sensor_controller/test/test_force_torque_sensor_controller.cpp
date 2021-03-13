// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#include "force_torque_sensor_controller/force_torque_sensor_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_force_torque_sensor_controller.hpp"

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

void ForceTorqueSensorControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void ForceTorqueSensorControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void ForceTorqueSensorControllerTest::SetUp()
{
  // initialize controller
  state_controller_ = std::make_unique<FriendForceTorqueSensorController>();
}

void ForceTorqueSensorControllerTest::TearDown()
{
  state_controller_.reset(nullptr);
}

void ForceTorqueSensorControllerTest::SetUpStateController()
{
  const auto result = state_controller_->init("force_torque_sensor_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(sensor_1_fx_state_);
  state_ifs.emplace_back(sensor_2_fx_state_);
  state_ifs.emplace_back(sensor_3_fx_state_);
  state_ifs.emplace_back(sensor_1_tz_state_);
  state_ifs.emplace_back(sensor_2_tz_state_);
  state_ifs.emplace_back(sensor_3_tz_state_);

  state_controller_->assign_interfaces({}, std::move(state_ifs));
}

TEST_F(ForceTorqueSensorControllerTest, ConfigureErrorTest)
{
  // sensor state not initialized yet
  ASSERT_TRUE(state_controller_->wrench_state_msg_.header.empty());
  ASSERT_TRUE(state_controller_->wrench_state_msg_.wrench.empty());

  // publishers not initialized yet
  ASSERT_FALSE(state_controller_->sensor_state_publisher_);

  // configure failed
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  SetUpStateController();
  // check state remains unchanged

  // sensor state still not initialized
  ASSERT_TRUE(state_controller_->wrench_state_msg_.header.empty());
  ASSERT_TRUE(state_controller_->wrench_state_msg_.wrench.empty());

  // publishers still not initialized
  ASSERT_FALSE(state_controller_->sensor_state_publisher_);
}

TEST_F(ForceTorqueSensorControllerTest, ConfigureSuccessTest)
{
  // sensor state not initialized yet
  ASSERT_THAT(state_controller_->wrench_state_msg_.header, IsEmpty());
  ASSERT_THAT(state_controller_->wrench_state_msg_.wrench, IsEmpty());

  // publishers not initialized yet
  ASSERT_FALSE(state_controller_->sensor_state_publisher_);

  SetUpStateController();
  // configure ok
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(state_controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const size_t NUM_SENSORS = sensor_names_.size();
  const std::vector<std::string> IF_NAMES = {"fx", "tz"};
  const size_t NUM_IFS = IF_NAMES.size();

  // sensor state initialized
  ASSERT_THAT(state_controller_->wrench_state_msg_.wrench, SizeIs(NUM_SENSORS));

  // publishers initialized
  ASSERT_TRUE(state_controller_->sensor_state_publisher_);
}

TEST_F(ForceTorqueSensorControllerTest, UpdateTest)
{
  SetUpStateController();

  auto node_state = state_controller_->configure();
  node_state = state_controller_->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorControllerTest, SensorStatePublishTest)
{
  SetUpStateController();

  auto node_state = state_controller_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_controller_->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");

  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    "sensor_states",
    10,
    subs_callback);

  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  geometry_msgs::msg::WrenchStamped sensor_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(sensor_state_msg, msg_info));

  const size_t NUM_SENSORS = sensor_names_.size();
  ASSERT_THAT(sensor_state_msg.header, SizeIs(NUM_SENSORS));
  // the order in the message may be different
  // we only check that all values in this test are present in the message
  // and that it is the same across the interfaces
  // for test purposes they are mapped to the same doubles
  ASSERT_THAT(sensor_state_msg.wrench, ElementsAreArray(sensor_values_));
}
