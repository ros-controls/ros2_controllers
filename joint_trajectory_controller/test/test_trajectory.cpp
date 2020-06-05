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
#include <vector>

#include "gtest/gtest.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

TEST(TestTrajectory, initialize_trajectory) {
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 2;
    empty_msg->header.stamp.nanosec = 2;
    const rclcpp::Time empty_time = empty_msg->header.stamp;
    auto traj = joint_trajectory_controller::Trajectory(empty_msg);

    trajectory_msgs::msg::JointTrajectoryPoint expected_point;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    traj.sample(rclcpp::Clock().now(), expected_point, start, end);

    EXPECT_EQ(traj.end(), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_EQ(empty_time, traj.time_from_start());
  }
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 0;
    empty_msg->header.stamp.nanosec = 0;
    const auto now = rclcpp::Clock().now();
    auto traj = joint_trajectory_controller::Trajectory(empty_msg);
    const auto traj_starttime = traj.time_from_start();

    trajectory_msgs::msg::JointTrajectoryPoint expected_point;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    traj.sample(rclcpp::Clock().now(), expected_point, start, end);

    EXPECT_EQ(traj.end(), start);
    EXPECT_EQ(traj.end(), end);
    const auto allowed_delta = 10000ll;
    EXPECT_LT(traj.time_from_start().nanoseconds() - now.nanoseconds(), allowed_delta);
  }
}

TEST(TestTrajectory, sample_trajectory) {
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp.sec = 0;
  full_msg->header.stamp.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions.push_back(1.0);
  p1.velocities.push_back(0.0);
  p1.accelerations.push_back(0.0);
  p1.time_from_start.sec = 1;
  p1.time_from_start.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions.push_back(2.0);
  p2.velocities.push_back(0.0);
  p2.accelerations.push_back(0.0);
  p2.time_from_start.sec = 2;
  p2.time_from_start.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions.push_back(3.0);
  p3.velocities.push_back(0.0);
  p3.accelerations.push_back(0.0);
  p3.time_from_start.sec = 3;
  p3.time_from_start.nanosec = 0;

  full_msg->points.push_back(p1);
  full_msg->points.push_back(p2);
  full_msg->points.push_back(p3);

  // set current state before trajectory msg was sent
  auto traj = joint_trajectory_controller::Trajectory(full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint current_point;
  current_point.time_from_start.sec = 0;
  current_point.time_from_start.nanosec = 0;
  current_point.positions.push_back(0.0);
  current_point.velocities.push_back(0.0);
  current_point.accelerations.push_back(0.0);

  const rclcpp::Time time_now = rclcpp::Clock().now();
  traj.set_point_before_trajectory_msg(time_now, current_point);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;
  traj.sample(time_now, expected_state, start, end);

  ASSERT_EQ(traj.begin(), start);
  ASSERT_EQ(traj.begin(), end);
  EXPECT_EQ(0.0, expected_state.positions[0]);

  // sample before trajectory starts
  traj.sample(time_now - rclcpp::Duration::from_seconds(0.5), expected_state, start, end);
  ASSERT_EQ(traj.begin(), start);
  ASSERT_EQ(traj.begin(), end);
  EXPECT_EQ(0.0, expected_state.positions[0]);

  traj.sample(time_now + rclcpp::Duration::from_seconds(0.5), expected_state, start, end);
  ASSERT_EQ(traj.begin(), start);
  ASSERT_EQ(traj.begin(), end);
  EXPECT_EQ(0.5, expected_state.positions[0]);

  traj.sample(time_now + rclcpp::Duration::from_seconds(1.0), expected_state, start, end);
  ASSERT_EQ(traj.begin(), start);
  ASSERT_EQ((++traj.begin()), end);
  EXPECT_EQ(1.0, expected_state.positions[0]);

  traj.sample(time_now + rclcpp::Duration::from_seconds(1.5), expected_state, start, end);
  ASSERT_EQ(traj.begin(), start);
  ASSERT_EQ((++traj.begin()), end);
  EXPECT_EQ(1.5, expected_state.positions[0]);

  traj.sample(time_now + rclcpp::Duration::from_seconds(2.5), expected_state, start, end);
  EXPECT_EQ(2.5, expected_state.positions[0]);

  traj.sample(time_now + rclcpp::Duration::from_seconds(3.0), expected_state, start, end);
  EXPECT_EQ(3.0, expected_state.positions[0]);

  // sample past given points
  traj.sample(time_now + rclcpp::Duration::from_seconds(3.125), expected_state, start, end);
  ASSERT_EQ((--traj.end()), start);
  ASSERT_EQ(traj.end(), end);
  EXPECT_EQ(3.0, expected_state.positions[0]);
}
