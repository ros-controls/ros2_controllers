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
#include <cmath>
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

// Floating-point value comparison threshold
const double EPS = 1e-8;

TEST(TestTrajectory, initialize_trajectory)
{
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

TEST(TestTrajectory, sample_trajectory_positions)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions.push_back(1.0);
  p1.time_from_start = rclcpp::Duration::from_seconds(1.0);
  full_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions.push_back(2.0);
  p2.time_from_start = rclcpp::Duration::from_seconds(2.0);
  full_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions.push_back(3.0);
  p3.time_from_start = rclcpp::Duration::from_seconds(3.0);
  full_msg->points.push_back(p3);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);

  // set current state before trajectory msg was sent
  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  double duration_first_seg = 1.0;
  double velocity = (p1.positions[0] - point_before_msg.positions[0]) / duration_first_seg;

  // sample at trajectory starting time
  {
    traj.sample(time_now, expected_state, start, end);
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample before time_now
  {
    bool result =
      traj.sample(time_now - rclcpp::Duration::from_seconds(0.5), expected_state, start, end);
    ASSERT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(0.5), expected_state, start, end);
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ(traj.begin(), end);
    double half_current_to_p1 = (point_before_msg.positions[0] + p1.positions[0]) * 0.5;
    EXPECT_NEAR(half_current_to_p1, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample 1s after msg
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(1.0), expected_state, start, end);
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ((++traj.begin()), end);
    EXPECT_NEAR(p1.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample 1.5s after msg
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(1.5), expected_state, start, end);
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ((++traj.begin()), end);
    double half_p1_to_p2 = (p1.positions[0] + p2.positions[0]) * 0.5;
    EXPECT_NEAR(half_p1_to_p2, expected_state.positions[0], EPS);
  }

  // sample 2.5s after msg
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(2.5), expected_state, start, end);
    double half_p2_to_p3 = (p2.positions[0] + p3.positions[0]) * 0.5;
    EXPECT_NEAR(half_p2_to_p3, expected_state.positions[0], EPS);
  }

  // sample 3s after msg
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(3.0), expected_state, start, end);
    EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
  }

  // sample past given points
  {
    traj.sample(time_now + rclcpp::Duration::from_seconds(3.125), expected_state, start, end);
    ASSERT_EQ((--traj.end()), start);
    ASSERT_EQ(traj.end(), end);
    EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
  }
}

TEST(TestTrajectory, interpolation_pos_vel)
{
  // taken from ros1_controllers QuinticSplineSegmentTest::PosVelEnpointsSampler

  // Start and end state taken from x^3 - 2x
  trajectory_msgs::msg::JointTrajectoryPoint start_state;
  start_state.time_from_start = rclcpp::Duration::from_seconds(1.0);
  start_state.positions.push_back(0.0);
  start_state.velocities.push_back(-2.0);
  start_state.accelerations.clear();

  trajectory_msgs::msg::JointTrajectoryPoint end_state;
  end_state.time_from_start = rclcpp::Duration::from_seconds(3.0);
  end_state.positions.push_back(4.0);
  end_state.velocities.push_back(10.0);
  end_state.accelerations.push_back(0.0);  // Should be ignored, start state does not specify it

  auto traj = joint_trajectory_controller::Trajectory();
  rclcpp::Time time_now(0);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;

  // sample at start_time
  {
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + start_state.time_from_start, expected_state);
    EXPECT_NEAR(start_state.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(start_state.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // Sample at mid-segment: Zero-crossing
  {
    auto t = rclcpp::Duration::from_seconds(std::sqrt(2.0));
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + start_state.time_from_start + t, expected_state);
    EXPECT_NEAR(0.0, expected_state.positions[0], EPS);
    EXPECT_NEAR(4.0, expected_state.velocities[0], EPS);
    EXPECT_NEAR(6.0 * std::sqrt(2.0), expected_state.accelerations[0], EPS);
  }

  // sample at end_time
  {
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + end_state.time_from_start, expected_state);
    EXPECT_NEAR(end_state.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(end_state.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(12.0, expected_state.accelerations[0], EPS);
  }
}

TEST(TestTrajectory, interpolation_pos_vel_accel)
{
  // taken from ros1_controllers QuinticSplineSegmentTest::PosVeAcclEnpointsSampler

  // Start and end state taken from x(x-1)(x-2)(x-3)(x-4) = x^5 -10x^4 + 35x^3 -50x^2 + 24x
  trajectory_msgs::msg::JointTrajectoryPoint start_state;
  start_state.time_from_start = rclcpp::Duration::from_seconds(1.0);
  start_state.positions.push_back(0.0);
  start_state.velocities.push_back(24.0);
  start_state.accelerations.push_back(-100.0);

  trajectory_msgs::msg::JointTrajectoryPoint end_state;
  end_state.time_from_start = rclcpp::Duration::from_seconds(3.0);
  end_state.positions.push_back(0.0);
  end_state.velocities.push_back(4.0);
  end_state.accelerations.push_back(0.0);

  auto traj = joint_trajectory_controller::Trajectory();
  rclcpp::Time time_now(0);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;

  // sample at start_time
  {
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + start_state.time_from_start, expected_state);
    EXPECT_NEAR(start_state.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(start_state.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(start_state.accelerations[0], expected_state.accelerations[0], EPS);
  }

  // Sample at mid-segment: Zero-crossing
  {
    auto t = rclcpp::Duration::from_seconds(1.0);
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + start_state.time_from_start + t, expected_state);
    EXPECT_NEAR(0.0, expected_state.positions[0], EPS);
    EXPECT_NEAR(-6.0, expected_state.velocities[0], EPS);
    EXPECT_NEAR(10.0, expected_state.accelerations[0], EPS);
  }

  // sample at end_time
  {
    traj.interpolate_between_points(
      time_now + start_state.time_from_start, start_state, time_now + end_state.time_from_start,
      end_state, time_now + end_state.time_from_start, expected_state);
    EXPECT_NEAR(end_state.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(end_state.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(end_state.accelerations[0], expected_state.accelerations[0], EPS);
  }
}
