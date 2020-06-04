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

#include <memory>
#include <string>

// TEST CASES
TEST_F(DiffDriveControllerTest, test_break_with_multiple_publishers)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());
  ASSERT_TRUE(wait_for_odom_msgs());

  auto old_odom = get_last_odom();

  // Create another cmd_vel publisher
  auto another_node = std::make_shared<rclcpp::Node>("another_node");
  another_node->set_parameter({"use_sim_time", true});
  auto another_cmd_pub = another_node->create_publisher<geometry_msgs::msg::TwistStamped>(
    std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/cmd_vel", 100);

  // Create a timer where both nodes publish cmd_vel
  auto & node = get_node();
  auto & clk = *node->get_clock();
  auto timer = node->create_wall_timer(100ms, [this, &another_cmd_pub, &clk]() {
    geometry_msgs::msg::Twist cmd_vel_1;
    cmd_vel_1.linear.x = 1.0;
    publish(cmd_vel_1);
    std::this_thread::sleep_for(30ms);

    geometry_msgs::msg::TwistStamped cmd_vel_2;
    cmd_vel_2.twist.linear.x = 2.0;
    cmd_vel_2.header.stamp = clk.now();
    another_cmd_pub->publish(cmd_vel_2);
  });

  // Wait for a while
  sim_sleep_for(5s);
  auto new_odom = get_last_odom();

  const double dx = new_odom->pose.pose.position.x - old_odom->pose.pose.position.x;
  const double dy = new_odom->pose.pose.position.y - old_odom->pose.pose.position.y;
  const double dz = new_odom->pose.pose.position.z - old_odom->pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx * dx + dy * dy), 0.0, POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf2::Matrix3x3(tf_quat_from_geom_quat(old_odom->pose.pose.orientation))
    .getRPY(roll_old, pitch_old, yaw_old);
  tf2::Matrix3x3(tf_quat_from_geom_quat(new_odom->pose.pose.orientation))
    .getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);

  timer.reset();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
