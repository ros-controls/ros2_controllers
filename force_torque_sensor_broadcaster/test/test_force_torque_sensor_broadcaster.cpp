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

/*
 * Author: Subhas Das, Denis Stogl
 */

#include <stddef.h>

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "force_torque_sensor_broadcaster/force_torque_sensor_broadcaster.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_force_torque_sensor_broadcaster.hpp"

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

void ForceTorqueSensorBroadcasterTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void ForceTorqueSensorBroadcasterTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void ForceTorqueSensorBroadcasterTest::SetUp()
{
  // initialize controller
  state_controller_ = std::make_unique<FriendForceTorqueSensorBroadcaster>();
}

void ForceTorqueSensorBroadcasterTest::TearDown()
{
  state_controller_.reset(nullptr);
}

void ForceTorqueSensorBroadcasterTest::SetUpStateController()
{
  const auto result = state_controller_->init("force_torque_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorNameParameterNotSet)
{
  SetUpStateController();

  // configure failed, 'sensor_name' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNamesParameterNotSet)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // configure failed, 'interface_names' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, FrameIdParameterNotSet)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorNameParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name' empty
  state_controller_->get_node()->set_parameter({"sensor_name", ""});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // set the 'frame_id'
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure failed, 'sensor_name' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNameParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names' empty
  state_controller_->get_node()->set_parameter({"interface_names", std::vector<std::string>()});

  // set the 'frame_id'
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure failed, 'interface_name' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, FrameIdParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // set the 'frame_id' empty
  state_controller_->get_node()->set_parameter({"frame_id", ""});

  // configure failed, 'frame_id' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, ConfigureParamsSuccess)
{
  SetUpStateController();

  // set the params 'sensor_name', 'interface_names' and 'frame_id'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure success
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, ActivateSuccess)
{
  SetUpStateController();

  // set the params 'sensor_name', 'interface_names' and 'frame_id'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure and activate success
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, UpdateTest)
{
  SetUpStateController();

  // set the params 'sensor_name', 'interface_names' and 'frame_id'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorStatePublishTest)
{
  SetUpStateController();

  // set the params 'sensor_name', 'interface_names' and 'frame_id'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // create a new subscriber
  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    "sensor_state",
    10,
    subs_callback);

  // call update to publish the test value
  ASSERT_EQ(state_controller_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  geometry_msgs::msg::WrenchStamped sensor_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(sensor_state_msg, msg_info));
}
