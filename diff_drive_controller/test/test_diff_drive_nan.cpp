// Copyright 2020 PAL Robotics S.L.
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
#include "test_common.h"

#include <gtest/gtest.h>
#include <std_srvs/srv/empty.hpp>

#include <limits>
#include <memory>

TEST_F(DiffDriveControllerTest, test_nan)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(2s);

  // send a command
  cmd_vel.linear.x = 0.1;
  sim_sleep_for(2s);

  // stop robot (will generate NaN)
  auto node = get_node();
  auto stop_diffbot_srv = node->create_client<std_srvs::srv::Empty>("stop_diffbot");
  stop_diffbot_srv->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()).wait();
  sim_sleep_for(2s);

  nav_msgs::msg::Odometry::ConstSharedPtr odom = get_last_odom();

  EXPECT_FALSE(std::isnan(odom->twist.twist.linear.x));
  EXPECT_FALSE(std::isnan(odom->twist.twist.angular.z));
  EXPECT_FALSE(std::isnan(odom->pose.pose.position.x));
  EXPECT_FALSE(std::isnan(odom->pose.pose.position.y));
  EXPECT_FALSE(std::isnan(odom->pose.pose.orientation.z));
  EXPECT_FALSE(std::isnan(odom->pose.pose.orientation.w));

  // start robot
  auto start_diffbot_srv = node->create_client<std_srvs::srv::Empty>("start_diffbot");
  start_diffbot_srv->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()).wait();
  sim_sleep_for(2s);

  odom = get_last_odom();

  EXPECT_FALSE(std::isnan(odom->twist.twist.linear.x));
  EXPECT_FALSE(std::isnan(odom->twist.twist.angular.z));
  EXPECT_FALSE(std::isnan(odom->pose.pose.position.x));
  EXPECT_FALSE(std::isnan(odom->pose.pose.position.y));
  EXPECT_FALSE(std::isnan(odom->pose.pose.orientation.z));
  EXPECT_FALSE(std::isnan(odom->pose.pose.orientation.w));
}

TEST_F(DiffDriveControllerTest, test_nan_cmd)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(100ms);

  // send NaN
  for (int i = 0; i < 10; ++i) {
    cmd_vel.linear.x = NAN;
    cmd_vel.angular.z = NAN;
    publish(cmd_vel);
    geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_out_msg = get_last_cmd_vel_out();
    EXPECT_FALSE(std::isnan(cmd_vel_out_msg->twist.linear.x));
    EXPECT_FALSE(std::isnan(cmd_vel_out_msg->twist.angular.z));
    sim_sleep_for(100ms);
  }

  nav_msgs::msg::Odometry::ConstSharedPtr odom = get_last_odom();
  EXPECT_DOUBLE_EQ(odom->twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(odom->pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odom->pose.pose.position.y, 0.0);

  geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_out_msg = get_last_cmd_vel_out();
  EXPECT_DOUBLE_EQ(cmd_vel_out_msg->twist.linear.x, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
