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
TEST_F(DiffDriveControllerTest, test_timeout)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());
  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  // give some time to the controller to react to the command
  sim_sleep_for(100ms);
  // get initial odom
  nav_msgs::msg::Odometry::ConstSharedPtr old_odom = get_last_odom();
  // send a velocity command of 1 m/s
  cmd_vel.linear.x = 1.0;
  publish(cmd_vel);
  // wait a bit
  sim_sleep_for(3s);

  nav_msgs::msg::Odometry::ConstSharedPtr new_odom = get_last_odom();

  // check if the robot has stopped after 0.5s,
  // thus covering less than 0.5s*1.0m.s-1 + some (big) tolerance
  EXPECT_LT(fabs(new_odom->pose.pose.position.x - old_odom->pose.pose.position.x), 0.8);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
