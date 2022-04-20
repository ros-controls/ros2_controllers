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

#include "test_lidar_sensor_broadcaster.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lidar_sensor_broadcaster/lidar_sensor_broadcaster.hpp"
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

void LidarSensorBroadcasterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void LidarSensorBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

void LidarSensorBroadcasterTest::SetUp()
{
  // initialize controller
  lidar_broadcaster_ = std::make_unique<FriendLidarSensorBroadcaster>();
}

void LidarSensorBroadcasterTest::TearDown() { lidar_broadcaster_.reset(nullptr); }

void LidarSensorBroadcasterTest::SetUpLidarBroadcaster()
{
  const auto result = lidar_broadcaster_->init("test_lidar_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(lidar_angle_min_);
  state_ifs.emplace_back(lidar_angle_max_);
  state_ifs.emplace_back(lidar_angle_increment_);
  state_ifs.emplace_back(lidar_time_increment_);
  state_ifs.emplace_back(lidar_scan_time_);
  state_ifs.emplace_back(lidar_range_min_);
  state_ifs.emplace_back(lidar_range_max_);
  // state_ifs.emplace_back(lidar_orientation_x_);
  // state_ifs.emplace_back(lidar_orientation_y_);
  // state_ifs.emplace_back(lidar_orientation_z_);
  // state_ifs.emplace_back(lidar_orientation_w_);
  // state_ifs.emplace_back(lidar_angular_velocity_x_);
  // state_ifs.emplace_back(lidar_angular_velocity_y_);
  // state_ifs.emplace_back(lidar_angular_velocity_z_);
  // state_ifs.emplace_back(lidar_linear_acceleration_x_);
  // state_ifs.emplace_back(lidar_linear_acceleration_y_);
  // state_ifs.emplace_back(lidar_linear_acceleration_z_);

  lidar_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void LidarSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::LaserScan & lidar_msg)
{
  // create a new subscriber
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::LaserScan::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::LaserScan>(
    "/test_lidar_sensor_broadcaster/lidar", 2, subs_callback); //10, subs_callback);

  // call update to publish the test value
  ASSERT_EQ(lidar_broadcaster_->update(), controller_interface::return_type::OK);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(lidar_msg, msg_info));
}

TEST_F(LidarSensorBroadcasterTest, SensorName_InterfaceNames_NotSet)
{
  SetUpLidarBroadcaster();

  // configure failed
  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(LidarSensorBroadcasterTest, SensorName_FrameId_NotSet)
{
  SetUpLidarBroadcaster();

  // set the 'interface_names'
  lidar_broadcaster_->get_node()->set_parameter(
    {"interface_names.height", "lidar_sensor/height"});
  lidar_broadcaster_->get_node()->set_parameter(
    {"interface_names.width", "lidar_sensor/width"});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(LidarSensorBroadcasterTest, InterfaceNames_FrameId_NotSet)
{
  SetUpLidarBroadcaster();

  // set the 'sensor_name'
  lidar_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(LidarSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpLidarBroadcaster();

  // set the 'sensor_name'
  lidar_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'frame_id'
  lidar_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(LidarSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpLidarBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  lidar_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  lidar_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure and activate success
  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(lidar_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(LidarSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpLidarBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  lidar_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  lidar_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(lidar_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(lidar_broadcaster_->update(), controller_interface::return_type::OK);
}

TEST_F(LidarSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpLidarBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  lidar_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  lidar_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(lidar_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(lidar_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::LaserScan lidar_msg;
  subscribe_and_get_message(lidar_msg);

  ASSERT_EQ(lidar_msg.header.frame_id, frame_id_);
  ASSERT_EQ(lidar_msg.angle_min, sensor_values_[0]);
  ASSERT_EQ(lidar_msg.angle_max, sensor_values_[1]);
  ASSERT_EQ(lidar_msg.angle_increment, sensor_values_[2]);
  ASSERT_EQ(lidar_msg.time_increment, sensor_values_[3]);
  ASSERT_EQ(lidar_msg.scan_time, sensor_values_[4]);
  ASSERT_EQ(lidar_msg.range_min, sensor_values_[5]);
  ASSERT_EQ(lidar_msg.range_max, sensor_values_[6]);

  // ASSERT_EQ(lidar_msg.orientation.x, sensor_values_[0]);
  // ASSERT_EQ(lidar_msg.orientation.y, sensor_values_[1]);
  // ASSERT_EQ(lidar_msg.orientation.z, sensor_values_[2]);
  // ASSERT_EQ(lidar_msg.orientation.w, sensor_values_[3]);
  // ASSERT_EQ(lidar_msg.angular_velocity.x, sensor_values_[4]);
  // ASSERT_EQ(lidar_msg.angular_velocity.y, sensor_values_[5]);
  // ASSERT_EQ(lidar_msg.angular_velocity.z, sensor_values_[6]);
  // ASSERT_EQ(lidar_msg.linear_acceleration.x, sensor_values_[7]);
  // ASSERT_EQ(lidar_msg.linear_acceleration.y, sensor_values_[8]);
  // ASSERT_EQ(lidar_msg.linear_acceleration.z, sensor_values_[9]);
}
