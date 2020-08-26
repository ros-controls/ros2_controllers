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
TEST_F(DiffDriveControllerTest, test_pub_cmd_vel_out_topic)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());
  // msgs are published in the same loop
  // thus if odom is published cmd_vel_out
  // should be as well (if enabled)
  ASSERT_TRUE(wait_for_odom_msgs());

  EXPECT_TRUE(is_publishing_cmd_vel_out());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(100ms);

  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  sim_sleep_for(100ms);

  EXPECT_TRUE(is_publishing_cmd_vel_out());

  auto cmd_vel_out_msg = get_last_cmd_vel_out();
  EXPECT_GT(fabs(cmd_vel_out_msg->twist.linear.x), 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
