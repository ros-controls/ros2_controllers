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
 * Authors: Subhas Das, Denis Stogl
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
  fts_broadcaster_ = std::make_unique<FriendForceTorqueSensorBroadcaster>();
}

void ForceTorqueSensorBroadcasterTest::TearDown()
{
  fts_broadcaster_.reset(nullptr);
}

void ForceTorqueSensorBroadcasterTest::SetUpStateController()
{
  const auto result = fts_broadcaster_->init("test_force_torque_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_InterfaceNames_NotSet)
{
  SetUpStateController();

  // configure failed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_FrameId_NotSet)
{
  SetUpStateController();

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_FrameId_NotSet)
{
  SetUpStateController();

  // set the 'sensor_name'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_InterfaceNames_Set)
{
  SetUpStateController();

  // set the 'sensor_name'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});

  // configure failed, both 'sensor_name' and 'interface_names' supplied
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_IsEmpty_InterfaceNames_NotSet)
{
  SetUpStateController();

  // set the 'sensor_name' empty
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", ""});

  // configure failed, 'sensor_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceName_IsEmpty_SensorName_NotSet)
{
  SetUpStateController();

  // set the 'interface_names' empty
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", ""});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", ""});

  // configure failed, 'interface_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpStateController();

  // set the 'sensor_name'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Configure_Success)
{
  SetUpStateController();

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});

  // set the 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpStateController();

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure and activate success
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Activate_Success)
{
  SetUpStateController();

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure and activate success
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpStateController();

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->update(), controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Update_Success)
{
  SetUpStateController();

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->update(), controller_interface::return_type::SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpStateController();

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", "dummy"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // create a new subscriber
  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test_force_torque_sensor_broadcaster/sensor_states",
    10,
    subs_callback);

  // call update to publish the test value
  ASSERT_EQ(fts_broadcaster_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  geometry_msgs::msg::WrenchStamped sensor_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(sensor_state_msg, msg_info));
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Publish_Success)
{
  SetUpStateController();

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fx"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "tz"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // create a new subscriber
  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    {
    };
  auto subscription = test_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test_force_torque_sensor_broadcaster/sensor_states",
    10,
    subs_callback);

  // call update to publish the test value
  ASSERT_EQ(fts_broadcaster_->update(), controller_interface::return_type::SUCCESS);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  geometry_msgs::msg::WrenchStamped sensor_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(sensor_state_msg, msg_info));
}
