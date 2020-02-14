// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "gtest/gtest.h"

#include "joint_trajectory_controller/trajectory.hpp"

#include "rclcpp/clock.hpp"

using namespace std::chrono_literals;

class TestTrajectory : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }
};

TEST_F(TestTrajectory, initialize_trajectory) {
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 2;
    empty_msg->header.stamp.nanosec = 2;
    rclcpp::Time empty_time = empty_msg->header.stamp;
    auto traj = ros_controllers::Trajectory(empty_msg);
    EXPECT_EQ(traj.end(), traj.sample(rclcpp::Clock().now()));
    EXPECT_EQ(empty_time, traj.time_from_start());
  }
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 0;
    empty_msg->header.stamp.nanosec = 0;
    auto now = rclcpp::Clock().now();
    auto traj = ros_controllers::Trajectory(empty_msg);
    auto traj_starttime = traj.time_from_start();
    EXPECT_EQ(traj.end(), traj.sample(rclcpp::Clock().now()));
    auto allowed_delta = 10000ll;
    EXPECT_LT(traj.time_from_start().nanoseconds() - now.nanoseconds(), allowed_delta);
  }
}

TEST_F(TestTrajectory, sample_trajectory) {
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp.sec = 0;
  full_msg->header.stamp.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions.push_back(1.0f);
  builtin_interfaces::msg::Duration d1;
  d1.sec = 1;
  d1.nanosec = 0;
  p1.time_from_start = d1;

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions.push_back(2.0f);
  builtin_interfaces::msg::Duration d2;
  d2.sec = 2;
  d2.nanosec = 0;
  p2.time_from_start = d2;

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions.push_back(3.0f);
  builtin_interfaces::msg::Duration d3;
  d3.sec = 3;
  d3.nanosec = 0;
  p3.time_from_start = d3;

  full_msg->points.push_back(p1);
  full_msg->points.push_back(p2);
  full_msg->points.push_back(p3);

  auto traj = ros_controllers::Trajectory(full_msg);

  auto sample_p1 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p1);
  EXPECT_EQ(1.0f, sample_p1->positions[0]);

  auto sample_p11 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p11);
  EXPECT_EQ(1.0f, sample_p11->positions[0]);

  std::this_thread::sleep_for(1s);

  auto sample_p2 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p1);
  EXPECT_EQ(2.0f, sample_p2->positions[0]);

  auto sample_p22 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p22);
  EXPECT_EQ(2.0f, sample_p22->positions[0]);

  std::this_thread::sleep_for(1s);

  auto sample_p3 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p1);
  EXPECT_EQ(3.0f, sample_p3->positions[0]);

  auto sample_p33 = traj.sample(rclcpp::Clock().now());
  ASSERT_NE(traj.end(), sample_p33);
  EXPECT_EQ(3.0f, sample_p33->positions[0]);

  std::this_thread::sleep_for(1s);

  auto sample_end = traj.sample(rclcpp::Clock().now());
  EXPECT_EQ(traj.end(), sample_end);

  auto sample_end_end = traj.sample(rclcpp::Clock().now());
  EXPECT_EQ(traj.end(), sample_end_end);
}

TEST_F(TestTrajectory, future_sample_trajectory) {
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Clock().now();
  full_msg->header.stamp.sec += 2;  // extra padding

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions.push_back(1.0f);
  builtin_interfaces::msg::Duration d1;
  d1.sec = 1;
  d1.nanosec = 0;
  p1.time_from_start = d1;

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions.push_back(2.0f);
  builtin_interfaces::msg::Duration d2;
  d2.sec = 2;
  d2.nanosec = 0;
  p2.time_from_start = d2;

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions.push_back(3.0f);
  builtin_interfaces::msg::Duration d3;
  d3.sec = 3;
  d3.nanosec = 0;
  p3.time_from_start = d3;

  full_msg->points.push_back(p1);
  full_msg->points.push_back(p2);
  full_msg->points.push_back(p3);

  auto traj = ros_controllers::Trajectory(full_msg);

  // sample for future point
  auto sample_p0 = traj.sample(rclcpp::Clock().now());
  ASSERT_EQ(traj.end(), sample_p0);
}
