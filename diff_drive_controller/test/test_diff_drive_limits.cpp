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

#include "test_common.hpp"

// TEST CASES
// NOTE: All tests assume a controller parameterized by
// linear:
//   x:
//     has_velocity_limits: true
//     min_velocity: -0.5
//     max_velocity: 1.0
//     has_acceleration_limits: true
//     min_acceleration: -0.5
//     max_acceleration: 1.0
//     has_jerk_limits: true
//     min_jerk: -5.0
//     max_jerk: 5.0
// angular:
//     z:
//     has_velocity_limits: true
//     min_velocity: -2.0
//     max_velocity: 2.0
//     has_acceleration_limits: true
//     min_acceleration: -2.0
//     max_acceleration: 2.0
//     has_jerk_limits: true
//     min_jerk: -10.0
//     max_jerk: 10.0

TEST_F(DiffDriveControllerTest, test_linear_jerk_limits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr oldest_odom = get_last_odom();
  ASSERT_NEAR(oldest_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(oldest_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(250ms);
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  sim_sleep_for(250ms);
  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if jerk is below limit of 5.0m.s-3
  EXPECT_LT(
    fabs(
      new_odom->twist.twist.linear.x - 2 * old_odom->twist.twist.linear.x +
      oldest_odom->twist.twist.linear.x),
    (5 * 0.25 * 0.25) + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.z - old_odom->twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, test_linear_acceleration_limits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  ASSERT_NEAR(old_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(old_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(500ms);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if acceleration is below limit of 1.0m.s-2
  EXPECT_LT(
    fabs(new_odom->twist.twist.linear.x - old_odom->twist.twist.linear.x),
    0.5 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.z - old_odom->twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, test_linear_velocity_limits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  ASSERT_NEAR(old_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(old_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(5s);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_NEAR(new_odom->twist.twist.linear.x, 1, VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.angular.z - old_odom->twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, test_angular_jerk_limits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr oldest_odom = get_last_odom();
  ASSERT_NEAR(oldest_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(oldest_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(250ms);
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  sim_sleep_for(250ms);
  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if jerk is below limit of 10.0rad.s-3
  EXPECT_LT(
    fabs(
      new_odom->twist.twist.angular.z - 2 * old_odom->twist.twist.angular.z +
      oldest_odom->twist.twist.angular.z),
    (10.0 * 0.25 * 0.25) + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.x - old_odom->twist.twist.linear.x), EPS);
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, testAngularAccelerationLimits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  ASSERT_NEAR(old_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(old_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(500ms);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if acceleration is below limit of 2.0rad.s-2
  EXPECT_LT(
    fabs(new_odom->twist.twist.angular.z - old_odom->twist.twist.angular.z),
    1.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.x - old_odom->twist.twist.linear.x), EPS);

  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, testAngularVelocityLimits)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(4s);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  ASSERT_NEAR(old_odom->twist.twist.linear.x, 0, EPS);
  ASSERT_NEAR(old_odom->twist.twist.angular.z, 0, EPS);
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  sim_sleep_for(5s);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if the robot speed is now 2.0rad.s-1, the limit
  EXPECT_NEAR(new_odom->twist.twist.angular.z, 2, VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom->twist.twist.linear.x - old_odom->twist.twist.linear.x), EPS);

  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
