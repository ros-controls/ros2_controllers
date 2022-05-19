// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "test_range_sensor_broadcaster.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "range_sensor_broadcaster/range_sensor_broadcaster.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using hardware_interface::LoanedStateInterface;

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
  return wait_set.wait().kind();
}

}  // namespace

void RangeSensorBroadcasterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void RangeSensorBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

void RangeSensorBroadcasterTest::SetUp()
{
  // initialize controller
  range_broadcaster_ = std::make_unique<FriendRangeSensorBroadcaster>();
}

void RangeSensorBroadcasterTest::TearDown() { range_broadcaster_.reset(nullptr); }

void RangeSensorBroadcasterTest::SetUpRangeBroadcaster()
{
  const auto result = range_broadcaster_->init("test_range_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(range_radiation_type_);
  state_ifs.emplace_back(range_field_of_view_);
  state_ifs.emplace_back(range_min_range_);
  state_ifs.emplace_back(range_max_range_);
  state_ifs.emplace_back(range_range_);

  range_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void RangeSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Range & range_msg)
{
  // create a new subscriber
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Range::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Range>(
    "/test_range_sensor_broadcaster/range", 2, subs_callback); //10, subs_callback);

  // call update to publish the test value
  ASSERT_EQ(range_broadcaster_->update(), controller_interface::return_type::OK);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(range_msg, msg_info));
}

TEST_F(RangeSensorBroadcasterTest, SensorName_InterfaceNames_NotSet)
{
  SetUpRangeBroadcaster();

  // configure failed
  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(RangeSensorBroadcasterTest, SensorName_FrameId_NotSet)
{
  SetUpRangeBroadcaster();

  // set the 'interface_names'
  range_broadcaster_->get_node()->set_parameter(
    {"interface_names.radiation_type", "range_sensor/radiation_type"});
  range_broadcaster_->get_node()->set_parameter(
    {"interface_names.field_of_view", "range_sensor/field_of_view"});
  range_broadcaster_->get_node()->set_parameter(
    {"interface_names.min_range", "range_sensor/min_range"});
  range_broadcaster_->get_node()->set_parameter(
    {"interface_names.max_range", "range_sensor/max_range"});
  range_broadcaster_->get_node()->set_parameter(
    {"interface_names.range", "range_sensor/range"});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(RangeSensorBroadcasterTest, InterfaceNames_FrameId_NotSet)
{
  SetUpRangeBroadcaster();

  // set the 'sensor_name'
  range_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(RangeSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpRangeBroadcaster();

  // set the 'sensor_name'
  range_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'frame_id'
  range_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(RangeSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpRangeBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  range_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  range_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure and activate success
  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(range_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(RangeSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpRangeBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  range_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  range_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(range_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(range_broadcaster_->update(), controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpRangeBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  range_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  range_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(range_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(range_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::Range range_msg;
  subscribe_and_get_message(range_msg);

  ASSERT_EQ(range_msg.header.frame_id, frame_id_);
  ASSERT_EQ(range_msg.radiation_type, sensor_values_[0]);
  ASSERT_EQ(range_msg.field_of_view, sensor_values_[1]);
  ASSERT_EQ(range_msg.min_range, sensor_values_[2]);
  ASSERT_EQ(range_msg.max_range, sensor_values_[3]);
  ASSERT_EQ(range_msg.range, sensor_values_[4]);
}
