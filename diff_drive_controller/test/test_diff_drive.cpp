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

#include <gtest/gtest.h>
#include <tf2_ros/transform_listener.h>

#include "test_common.h"

namespace
{
}  // namespace

TEST_F(DiffDriveControllerTest, test_forward)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());
  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(100ms);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  // send a velocity command of 0.1 m/s
  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  // wait for 10s std::this_thread::sleep_for(10'000ms);
  sim_sleep_for(10'000ms);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  const rclcpp::Duration actual_elapsed_time =
    rclcpp::Time(new_odom->header.stamp) - rclcpp::Time(old_odom->header.stamp);

  const double expected_distance = cmd_vel.linear.x * actual_elapsed_time.seconds();

  // check if the robot traveled 1 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom->pose.pose.position.x - old_odom->pose.pose.position.x;
  const double dy = new_odom->pose.pose.position.y - old_odom->pose.pose.position.y;
  const double dz = new_odom->pose.pose.position.z - old_odom->pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx * dx + dy * dy), expected_distance, POSITION_TOLERANCE);
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
  EXPECT_NEAR(fabs(new_odom->twist.twist.linear.x), cmd_vel.linear.x, EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom->twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.z), EPS);
}

TEST_F(DiffDriveControllerTest, test_turn)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);

  sim_sleep_for(100ms);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  // send a velocity command
  cmd_vel.angular.z = M_PI / 10.0;
  publish(cmd_vel);
  // wait for 10s
  sim_sleep_for(10'000ms);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if the robot rotated PI around z, changes in the other fields should be ~~0
  EXPECT_LT(fabs(new_odom->pose.pose.position.x - old_odom->pose.pose.position.x), EPS);
  EXPECT_LT(fabs(new_odom->pose.pose.position.y - old_odom->pose.pose.position.y), EPS);
  EXPECT_LT(fabs(new_odom->pose.pose.position.z - old_odom->pose.pose.position.z), EPS);

  const rclcpp::Duration actual_elapsed_time =
    rclcpp::Time(new_odom->header.stamp) - rclcpp::Time(old_odom->header.stamp);
  const double expected_rotation = cmd_vel.angular.z * actual_elapsed_time.seconds();

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf2::Matrix3x3(tf_quat_from_geom_quat(old_odom->pose.pose.orientation))
    .getRPY(roll_old, pitch_old, yaw_old);
  tf2::Matrix3x3(tf_quat_from_geom_quat(new_odom->pose.pose.orientation))
    .getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), expected_rotation, ORIENTATION_TOLERANCE);

  EXPECT_LT(fabs(new_odom->twist.twist.linear.x), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom->twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.y), EPS);
  EXPECT_NEAR(
    fabs(new_odom->twist.twist.angular.z), expected_rotation / actual_elapsed_time.seconds(), EPS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
