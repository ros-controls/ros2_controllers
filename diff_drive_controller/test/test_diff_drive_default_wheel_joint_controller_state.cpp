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
TEST_F(DiffDriveControllerTest, test_default_joint_trajectory_controller_state_topic)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  EXPECT_FALSE(is_publishing_joint_trajectory_controller_state());

  // zero everything before test
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  sim_sleep_for(100ms);

  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  sim_sleep_for(100ms);

  EXPECT_FALSE(is_publishing_joint_trajectory_controller_state());
}
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
