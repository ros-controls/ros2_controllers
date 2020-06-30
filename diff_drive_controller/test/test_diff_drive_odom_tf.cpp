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

#include <gtest/gtest.h>
#include <tf2_ros/transform_listener.h>

// TEST CASES
TEST_F(DiffDriveControllerTest, test_no_odom_frame)
{
  // wait for ROS
  ASSERT_TRUE(wait_for_controller());

  // check the odom frame does not exist
  EXPECT_FALSE(
    get_tf_buffer()->canTransform(
      DEFAULT_BASE_FRAME_ID, DEFAULT_ODOM_FRAME_ID, rclcpp::Time(0), 2'000ms));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
