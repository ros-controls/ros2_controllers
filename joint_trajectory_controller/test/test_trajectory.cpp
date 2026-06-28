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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <gmock/gmock.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace joint_trajectory_controller::interpolation_methods;  // NOLINT
using namespace std::chrono_literals;

namespace
{
// Floating-point value comparison threshold
const double EPS = 1e-8;
}  // namespace

TEST(TestTrajectory, initialize_trajectory)
{
  auto clock = rclcpp::Clock(RCL_STEADY_TIME);
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 2;
    empty_msg->header.stamp.nanosec = 2;
    const rclcpp::Time empty_time = empty_msg->header.stamp;
    auto traj = joint_trajectory_controller::Trajectory(empty_msg);
    EXPECT_EQ(0, traj.last_sample_index());

    trajectory_msgs::msg::JointTrajectoryPoint expected_point;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    traj.sample(clock.now(), DEFAULT_INTERPOLATION, expected_point, start, end);
    EXPECT_EQ(0, traj.last_sample_index());

    EXPECT_EQ(traj.end(), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_EQ(empty_time, traj.time_from_start());
  }
  {
    auto empty_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    empty_msg->header.stamp.sec = 0;
    empty_msg->header.stamp.nanosec = 0;
    const auto now = clock.now();
    auto traj = joint_trajectory_controller::Trajectory(empty_msg);
    const auto traj_starttime = traj.time_from_start();

    trajectory_msgs::msg::JointTrajectoryPoint expected_point;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    traj.sample(clock.now(), DEFAULT_INTERPOLATION, expected_point, start, end);
    EXPECT_EQ(0, traj.last_sample_index());

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
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample before time_now
  {
    bool result = traj.sample(
      time_now - rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ(traj.begin(), end);
    double half_current_to_p1 = (point_before_msg.positions[0] + p1.positions[0]) * 0.5;
    EXPECT_NEAR(half_current_to_p1, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample 1s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ((++traj.begin()), end);
    EXPECT_NEAR(p1.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample 1.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ((++traj.begin()), end);
    double half_p1_to_p2 = (p1.positions[0] + p2.positions[0]) * 0.5;
    EXPECT_NEAR(half_p1_to_p2, expected_state.positions[0], EPS);
  }

  // sample 2.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(2.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(1, traj.last_sample_index());
    double half_p2_to_p3 = (p2.positions[0] + p3.positions[0]) * 0.5;
    EXPECT_NEAR(half_p2_to_p3, expected_state.positions[0], EPS);
  }

  // sample 3s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(2, traj.last_sample_index());
    EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
  }

  // sample past given points
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.125), DEFAULT_INTERPOLATION, expected_state,
      start, end);
    EXPECT_EQ(2, traj.last_sample_index());
    ASSERT_EQ((--traj.end()), start);
    ASSERT_EQ(traj.end(), end);
    EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
  }

  // sample long past given points for same trajectory, it should receive the same end point
  // so later in the query_state_service we set it to failure
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(30.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(2, traj.last_sample_index());
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

TEST(TestTrajectory, sample_trajectory_velocity_with_interpolation)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  // definitions
  double time_first_seg = 1.0;
  double time_second_seg = 2.0;
  double time_third_seg = 3.0;
  double velocity_first_seg = 1.0;
  double velocity_second_seg = 2.0;
  double velocity_third_seg = 1.0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.velocities.push_back(velocity_first_seg);
  p1.time_from_start = rclcpp::Duration::from_seconds(time_first_seg);
  full_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.velocities.push_back(velocity_second_seg);
  p2.time_from_start = rclcpp::Duration::from_seconds(time_second_seg);
  full_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.velocities.push_back(velocity_third_seg);
  p3.time_from_start = rclcpp::Duration::from_seconds(time_third_seg);
  full_msg->points.push_back(p3);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);
  point_before_msg.velocities.push_back(0.0);

  // set current state before trajectory msg was sent
  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // sample at trajectory starting time
  {
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(traj.last_sample_index(), 0);
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(point_before_msg.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR((velocity_first_seg / time_first_seg), expected_state.accelerations[0], EPS);
  }

  // sample before time_now
  {
    bool result = traj.sample(
      time_now - rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    double half_current_to_p1 =
      point_before_msg.positions[0] +
      (point_before_msg.velocities[0] +
       ((point_before_msg.velocities[0] + p1.velocities[0]) / 2 - point_before_msg.velocities[0]) /
         2) *
        0.5;
    EXPECT_NEAR(half_current_to_p1, expected_state.positions[0], EPS);
    EXPECT_NEAR(p1.velocities[0] / 2, expected_state.velocities[0], EPS);
    EXPECT_NEAR((velocity_first_seg / time_first_seg), expected_state.accelerations[0], EPS);
  }

  // sample 1s after msg
  double position_first_seg =
    point_before_msg.positions[0] + (0.0 + p1.velocities[0]) / 2 * time_first_seg;
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ((++traj.begin()), end);
    EXPECT_NEAR(position_first_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(p1.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(
      (velocity_second_seg - velocity_first_seg / (time_second_seg - time_first_seg)),
      expected_state.accelerations[0], EPS);
  }

  // sample 1.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ((++traj.begin()), end);
    double half_p1_to_p2 =
      position_first_seg +
      (p1.velocities[0] + ((p1.velocities[0] + p2.velocities[0]) / 2 - p1.velocities[0]) / 2) * 0.5;
    EXPECT_NEAR(half_p1_to_p2, expected_state.positions[0], EPS);
    double half_p1_to_p2_vel = (p1.velocities[0] + p2.velocities[0]) / 2;
    EXPECT_NEAR(half_p1_to_p2_vel, expected_state.velocities[0], EPS);
    EXPECT_NEAR(
      (velocity_second_seg - velocity_first_seg / (time_second_seg - time_first_seg)),
      expected_state.accelerations[0], EPS);
  }

  // sample 2s after msg
  double position_second_seg = position_first_seg + (p1.velocities[0] + p2.velocities[0]) / 2 *
                                                      (time_second_seg - time_first_seg);
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(2), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(1, traj.last_sample_index());
    EXPECT_EQ((++traj.begin()), start);
    EXPECT_EQ((--traj.end()), end);
    EXPECT_NEAR(position_second_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(p2.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(
      (velocity_third_seg - velocity_second_seg / (time_third_seg - time_second_seg)),
      expected_state.accelerations[0], EPS);
  }

  // sample 2.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(2.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(1, traj.last_sample_index());
    EXPECT_EQ((++traj.begin()), start);
    EXPECT_EQ((--traj.end()), end);
    double half_p2_to_p3 =
      position_second_seg +
      (p2.velocities[0] + ((p2.velocities[0] + p3.velocities[0]) / 2 - p2.velocities[0]) / 2) * 0.5;
    EXPECT_NEAR(half_p2_to_p3, expected_state.positions[0], EPS);
    double half_p2_to_p3_vel = (p2.velocities[0] + p3.velocities[0]) / 2;
    EXPECT_NEAR(half_p2_to_p3_vel, expected_state.velocities[0], EPS);
    EXPECT_NEAR(
      (velocity_third_seg - velocity_second_seg / (time_third_seg - time_second_seg)),
      expected_state.accelerations[0], EPS);
  }

  // sample 3s after msg
  double position_third_seg = position_second_seg + (p2.velocities[0] + p3.velocities[0]) / 2 *
                                                      (time_third_seg - time_second_seg);
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(2, traj.last_sample_index());
    EXPECT_EQ((--traj.end()), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_NEAR(position_third_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(p3.velocities[0], expected_state.velocities[0], EPS);
    // the goal is reached so no acceleration anymore
    EXPECT_NEAR(0, expected_state.accelerations[0], EPS);
  }

  // sample past given points - movement virtually stops
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.125), DEFAULT_INTERPOLATION, expected_state,
      start, end);
    EXPECT_EQ(2, traj.last_sample_index());
    EXPECT_EQ((--traj.end()), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_NEAR(position_third_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(p3.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }
}

// This test is added because previous one behaved strange if
// "point_before_msg.velocities.push_back(0.0);" was not defined
TEST(TestTrajectory, sample_trajectory_velocity_with_interpolation_strange_without_vel)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  // definitions
  double time_first_seg = 1.0;
  double time_second_seg = 2.0;
  double time_third_seg = 3.0;
  double velocity_first_seg = 1.0;
  double velocity_second_seg = 2.0;
  double velocity_third_seg = 1.0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.velocities.push_back(velocity_first_seg);
  p1.time_from_start = rclcpp::Duration::from_seconds(time_first_seg);
  full_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.velocities.push_back(velocity_second_seg);
  p2.time_from_start = rclcpp::Duration::from_seconds(time_second_seg);
  full_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.velocities.push_back(velocity_third_seg);
  p3.time_from_start = rclcpp::Duration::from_seconds(time_third_seg);
  full_msg->points.push_back(p3);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);

  // set current state before trajectory msg was sent
  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // sample at trajectory starting time
  {
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(0.0, expected_state.velocities[0], EPS);
    // is 0 because point_before_msg does not have velocity defined
    EXPECT_NEAR(1.0, expected_state.accelerations[0], EPS);
  }

  // sample before time_now
  {
    bool result = traj.sample(
      time_now - rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    //     double half_current_to_p1 = point_before_msg.positions[0] +
    //     (point_before_msg.velocities[0] +
    //     ((point_before_msg.velocities[0] + p1.velocities[0]) / 2 -
    //     point_before_msg.velocities[0]) / 2) * 0.5;
    double half_current_to_p1 =
      point_before_msg.positions[0] + (0.0 + ((0.0 + p1.velocities[0]) / 2 - 0.0) / 2) * 0.5;
    EXPECT_NEAR(half_current_to_p1, expected_state.positions[0], EPS);
    EXPECT_NEAR(p1.velocities[0] / 2, expected_state.velocities[0], EPS);
    EXPECT_NEAR(1.0, expected_state.accelerations[0], EPS);
  }

  // sample 1s after msg
  double position_first_seg =
    point_before_msg.positions[0] + (0.0 + p1.velocities[0]) / 2 * time_first_seg;
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ((++traj.begin()), end);
    EXPECT_NEAR(position_first_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(p1.velocities[0], expected_state.velocities[0], EPS);
    EXPECT_NEAR(1.0, expected_state.accelerations[0], EPS);
  }
}

TEST(TestTrajectory, sample_trajectory_acceleration_with_interpolation)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  // definitions
  double time_first_seg = 1.0;
  double time_second_seg = 2.0;
  double time_third_seg = 3.0;
  double acceleration_first_seg = 1.0;
  double acceleration_second_seg = 2.0;
  double acceleration_third_seg = 1.0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.accelerations.push_back(acceleration_first_seg);
  p1.time_from_start = rclcpp::Duration::from_seconds(time_first_seg);
  full_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.accelerations.push_back(acceleration_second_seg);
  p2.time_from_start = rclcpp::Duration::from_seconds(time_second_seg);
  full_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.accelerations.push_back(acceleration_third_seg);
  p3.time_from_start = rclcpp::Duration::from_seconds(time_third_seg);
  full_msg->points.push_back(p3);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);
  point_before_msg.velocities.push_back(0.0);

  // set current state before trajectory msg was sent
  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // sample at trajectory starting time
  {
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(0.0, expected_state.velocities[0], EPS);
    // is 0 because point_before_msg does not have velocity defined
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // sample before time_now
  {
    bool result = traj.sample(
      time_now - rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(result, false);
  }

  // Sample only on points testing of intermediate values is too complex and not necessary

  // sample 1s after msg
  double velocity_first_seg =
    point_before_msg.velocities[0] + (0.0 + p1.accelerations[0]) / 2 * time_first_seg;
  double position_first_seg =
    point_before_msg.positions[0] + (0.0 + velocity_first_seg) / 2 * time_first_seg;
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ((++traj.begin()), end);
    EXPECT_NEAR(position_first_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity_first_seg, expected_state.velocities[0], EPS);
    EXPECT_NEAR(p1.accelerations[0], expected_state.accelerations[0], EPS);
  }

  // sample 2s after msg
  double velocity_second_seg = velocity_first_seg + (p1.accelerations[0] + p2.accelerations[0]) /
                                                      2 * (time_second_seg - time_first_seg);
  double position_second_seg = position_first_seg + (velocity_first_seg + velocity_second_seg) / 2 *
                                                      (time_second_seg - time_first_seg);
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(2), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(1, traj.last_sample_index());
    EXPECT_EQ((++traj.begin()), start);
    EXPECT_EQ((--traj.end()), end);
    EXPECT_NEAR(position_second_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity_second_seg, expected_state.velocities[0], EPS);
    EXPECT_NEAR(p2.accelerations[0], expected_state.accelerations[0], EPS);
  }

  // sample 3s after msg
  double velocity_third_seg = velocity_second_seg + (p2.accelerations[0] + p3.accelerations[0]) /
                                                      2 * (time_third_seg - time_second_seg);
  double position_third_seg = position_second_seg + (velocity_second_seg + velocity_third_seg) / 2 *
                                                      (time_third_seg - time_second_seg);
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(2, traj.last_sample_index());
    EXPECT_EQ((--traj.end()), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_NEAR(position_third_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity_third_seg, expected_state.velocities[0], EPS);
    EXPECT_NEAR(p3.accelerations[0], expected_state.accelerations[0], EPS);
  }

  // sample past given points - movement virtually stops
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.125), DEFAULT_INTERPOLATION, expected_state,
      start, end);
    EXPECT_EQ(2, traj.last_sample_index());
    EXPECT_EQ((--traj.end()), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_NEAR(position_third_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity_third_seg, expected_state.velocities[0], EPS);
    EXPECT_NEAR(p3.accelerations[0], expected_state.accelerations[0], EPS);
  }
}

TEST(TestTrajectory, fill_point_before_with_same_degree_as_traj)
{
  auto full_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  full_msg->header.stamp = rclcpp::Time(0);

  // definitions
  double time_first_seg = 1.0;
  double time_second_seg = 2.0;
  double position_first_seg = 1.0;
  double position_second_seg = 2.0;
  double velocity_first_seg = 0.0;
  double velocity_second_seg = 0.0;
  double acceleration_first_seg = 0.0;
  double acceleration_second_seg = 0.0;

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.time_from_start = rclcpp::Duration::from_seconds(time_first_seg);
  p1.positions.push_back(position_first_seg);
  p1.velocities.push_back(velocity_first_seg);
  p1.accelerations.push_back(acceleration_first_seg);
  full_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.time_from_start = rclcpp::Duration::from_seconds(time_second_seg);
  p2.positions.push_back(position_second_seg);
  p2.velocities.push_back(velocity_second_seg);
  p2.accelerations.push_back(acceleration_second_seg);
  full_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);

  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, full_msg);

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // sample at trajectory starting time
  // Since the trajectory defines positions, velocities, and accelerations, we expect quintic
  // spline interpolation. Due to the unspecified initial acceleration, it should be zero.
  {
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(0, traj.last_sample_index());
    ASSERT_EQ(traj.begin(), start);
    ASSERT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(0.0, expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }
}

TEST(TestTrajectory, skip_interpolation)
{
  // Simple passthrough without extra interpolation
  {
    const InterpolationMethod no_interpolation = InterpolationMethod::NONE;

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

    // sample at trajectory starting time
    {
      traj.sample(time_now, no_interpolation, expected_state, start, end);
      EXPECT_EQ(0, traj.last_sample_index());
      ASSERT_EQ(traj.begin(), start);
      ASSERT_EQ(traj.begin(), end);
      EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
      // There were no vels/accels in the input, so they should remain empty
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.velocities.size());
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.accelerations.size());
    }

    // sample before time_now
    {
      bool result = traj.sample(
        time_now - rclcpp::Duration::from_seconds(0.5), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(0, traj.last_sample_index());
      ASSERT_EQ(result, false);
    }

    // sample 0.5s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(0.5), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(0, traj.last_sample_index());
      ASSERT_EQ(traj.begin(), start);
      ASSERT_EQ(traj.begin(), end);
      // For passthrough, this should just return the first waypoint
      EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
      // There were no vels/accels in the input, so they should remain empty
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.velocities.size());
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.accelerations.size());
    }

    // sample 1s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(1.0), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(0, traj.last_sample_index());
      ASSERT_EQ(traj.begin(), start);
      ASSERT_EQ((++traj.begin()), end);
      EXPECT_NEAR(p2.positions[0], expected_state.positions[0], EPS);
      // There were no vels/accels in the input, so they should remain empty
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.velocities.size());
      EXPECT_EQ(
        static_cast<std::make_unsigned<decltype(0)>::type>(0), expected_state.accelerations.size());
    }

    // sample 1.5s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(1.5), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(0, traj.last_sample_index());
      ASSERT_EQ(traj.begin(), start);
      ASSERT_EQ((++traj.begin()), end);
      EXPECT_NEAR(p2.positions[0], expected_state.positions[0], EPS);
    }

    // sample 2.5s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(2.5), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(1, traj.last_sample_index());
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }

    // sample 3s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(3.0), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(2, traj.last_sample_index());
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }

    // sample past given points
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(3.125), no_interpolation, expected_state, start,
        end);
      EXPECT_EQ(2, traj.last_sample_index());
      ASSERT_EQ((--traj.end()), start);
      ASSERT_EQ(traj.end(), end);
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }
  }
}

TEST(TestTrajectory, update_trajectory)
{
  // Verify that sampling works correctly after updating with a new trajectory
  auto first_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  first_msg->header.stamp = rclcpp::Time(0);

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions.push_back(1.0);
  p1.time_from_start = rclcpp::Duration::from_seconds(1.0);
  first_msg->points.push_back(p1);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions.push_back(2.0);
  p2.time_from_start = rclcpp::Duration::from_seconds(2.0);
  first_msg->points.push_back(p2);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions.push_back(3.0);
  p3.time_from_start = rclcpp::Duration::from_seconds(3.0);
  first_msg->points.push_back(p3);

  trajectory_msgs::msg::JointTrajectoryPoint point_before_msg;
  point_before_msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
  point_before_msg.positions.push_back(0.0);

  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, point_before_msg, first_msg);
  EXPECT_EQ(0, traj.last_sample_index());

  trajectory_msgs::msg::JointTrajectoryPoint expected_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // Sample at starting time
  traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
  EXPECT_EQ(0, traj.last_sample_index());

  // Sample 2.5s after msg
  traj.sample(
    time_now + rclcpp::Duration::from_seconds(2.5), DEFAULT_INTERPOLATION, expected_state, start,
    end);
  EXPECT_EQ(1, traj.last_sample_index());

  // Update trajectory
  auto snd_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  snd_msg->header.stamp = rclcpp::Time(0);

  snd_msg->points.push_back(p1);
  snd_msg->points.push_back(p2);
  snd_msg->points.push_back(p3);

  traj.update(snd_msg);

  // Sample at starting time
  {
    traj.sample(time_now, DEFAULT_INTERPOLATION, expected_state, start, end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(traj.begin(), end);
    EXPECT_NEAR(point_before_msg.positions[0], expected_state.positions[0], EPS);
    EXPECT_NEAR(
      (p1.positions[0] - point_before_msg.positions[0]), expected_state.velocities[0], EPS);
    EXPECT_NEAR(0.0, expected_state.accelerations[0], EPS);
  }

  // Sample 1.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(1.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_EQ(0, traj.last_sample_index());
    EXPECT_EQ(traj.begin(), start);
    EXPECT_EQ(std::next(traj.begin()), end);
    EXPECT_NEAR((p1.positions[0] + p2.positions[0]) / 2, expected_state.positions[0], EPS);
  }
}

TEST(TestWrapAroundJoint, no_wraparound)
{
  const std::vector<double> initial_position(3, 0.);
  std::vector<double> next_position(3, M_PI * 3. / 2.);

  std::vector<double> current_position(initial_position);
  std::vector<bool> joints_angle_wraparound(3, false);
  joint_trajectory_controller::wraparound_joint(
    current_position, next_position, joints_angle_wraparound);
  EXPECT_EQ(current_position[0], initial_position[0]);
  EXPECT_EQ(current_position[1], initial_position[1]);
  EXPECT_EQ(current_position[2], initial_position[2]);
}

TEST(TestWrapAroundJoint, wraparound_single_joint)
{
  const std::vector<double> initial_position(3, 0.);
  std::vector<double> next_position(3, M_PI * 3. / 2.);

  std::vector<double> current_position(initial_position);
  std::vector<bool> joints_angle_wraparound{true, false, false};
  joint_trajectory_controller::wraparound_joint(
    current_position, next_position, joints_angle_wraparound);
  EXPECT_EQ(current_position[0], initial_position[0] + 2 * M_PI);
  EXPECT_EQ(current_position[1], initial_position[1]);
  EXPECT_EQ(current_position[2], initial_position[2]);
}

TEST(TestWrapAroundJoint, wraparound_all_joints)
{
  const std::vector<double> initial_position(3, 0.);
  std::vector<double> next_position(3, M_PI * 3. / 2.);

  std::vector<double> current_position(initial_position);
  std::vector<bool> joints_angle_wraparound(3, true);
  joint_trajectory_controller::wraparound_joint(
    current_position, next_position, joints_angle_wraparound);
  EXPECT_EQ(current_position[0], initial_position[0] + 2 * M_PI);
  EXPECT_EQ(current_position[1], initial_position[1] + 2 * M_PI);
  EXPECT_EQ(current_position[2], initial_position[2] + 2 * M_PI);
}

TEST(TestWrapAroundJoint, wraparound_all_joints_no_offset)
{
  const std::vector<double> initial_position(3, 0.);
  std::vector<double> next_position(3, M_PI * 3. / 2.);

  std::vector<double> current_position(next_position);
  std::vector<bool> joints_angle_wraparound(3, true);
  joint_trajectory_controller::wraparound_joint(
    current_position, next_position, joints_angle_wraparound);
  EXPECT_EQ(current_position[0], next_position[0]);
  EXPECT_EQ(current_position[1], next_position[1]);
  EXPECT_EQ(current_position[2], next_position[2]);
}
// Regression test for #2282: with a velocity-only trajectory whose last
// segment is skipped (sample_time advances past the last segment without
// entering it, e.g. controller rate slower than the last segment duration),
// the last point's positions never get deduced from velocities. Before the
// fix, the "whole animation has played out" branch would copy that point
// into output_state with empty positions, which then segfaults later in
// JointTrajectoryController::compute_error_for_joint.
TEST(TestTrajectory, sample_velocity_only_skipped_last_segment_deduces_positions)
{
  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.time_from_start = rclcpp::Duration::from_seconds(1.0);
  p1.positions = {0.0};
  p1.velocities = {1.0};

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.time_from_start = rclcpp::Duration::from_seconds(2.0);
  // intentionally velocity-only; positions will be deduced
  p2.velocities = {1.0};

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.time_from_start = rclcpp::Duration::from_seconds(2.001);
  // intentionally velocity-only and tiny duration to model the skipped-last-segment case
  p3.velocities = {1.0};

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg->points = {p1, p2, p3};

  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, p1, traj_msg);

  trajectory_msgs::msg::JointTrajectoryPoint output_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // First sample at time_now anchors trajectory_start_time_ = time_now (header
  // stamp left default so the sample() code path that sets it on first call
  // fires). Second sample inside segment 0 so p2's positions get deduced from
  // velocities.
  traj.sample(time_now, DEFAULT_INTERPOLATION, output_state, start, end);
  traj.sample(
    time_now + rclcpp::Duration::from_seconds(1.5), DEFAULT_INTERPOLATION, output_state, start,
    end);
  ASSERT_FALSE(output_state.positions.empty());

  // Now sample past the end: segment 1 (between p2 and p3, 1 ms) is skipped
  // entirely, leaving p3.positions empty without the fix.
  const bool ok = traj.sample(
    time_now + rclcpp::Duration::from_seconds(3.0), DEFAULT_INTERPOLATION, output_state, start,
    end);

  EXPECT_TRUE(ok);
  ASSERT_FALSE(output_state.positions.empty())
    << "sample() must not return success with empty positions; would segfault "
       "in JointTrajectoryController::compute_error_for_joint";
  EXPECT_EQ(output_state.positions.size(), p1.positions.size());
}
// Regression test for #2282 pathological case: every point in the trajectory
// is velocity-only and sample_time is so far past the end that no segment
// was ever entered to deduce positions. With no usable position chain, the
// "whole animation has played out" branch must return false rather than
// copy empty positions into output_state.
TEST(TestTrajectory, sample_velocity_only_all_segments_skipped_returns_false)
{
  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.time_from_start = rclcpp::Duration::from_seconds(0.001);
  p1.velocities = {1.0};

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.time_from_start = rclcpp::Duration::from_seconds(0.002);
  p2.velocities = {1.0};

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg->points = {p1, p2};

  trajectory_msgs::msg::JointTrajectoryPoint state_before;
  // intentionally empty positions on the prior-state too, to force the
  // "no usable position anywhere" case
  state_before.velocities = {1.0};

  const rclcpp::Time time_now = rclcpp::Clock().now();
  auto traj = joint_trajectory_controller::Trajectory(time_now, state_before, traj_msg);

  trajectory_msgs::msg::JointTrajectoryPoint output_state;
  joint_trajectory_controller::TrajectoryPointConstIter start, end;

  // Anchor trajectory_start_time_ = time_now via a first sample. With
  // state_before.positions also empty, no segment will ever produce non-empty
  // positions, so the "whole animation has played out" branch must return false.
  traj.sample(time_now, DEFAULT_INTERPOLATION, output_state, start, end);
  const bool ok = traj.sample(
    time_now + rclcpp::Duration::from_seconds(10.0), DEFAULT_INTERPOLATION, output_state, start,
    end);

  EXPECT_FALSE(ok)
    << "sample() must return false when it cannot produce a non-empty positions vector";
}

// fill_cubic_spline_velocities (the positions->velocities sibling of
// deduce_from_derivatives): solves the knot velocities that turn a positions-
// only trajectory into a global cubic spline (C2).

namespace
{
// Build a positions-only chunk; positions_per_point[i] holds one position per
// joint for waypoint i, spaced dt apart.
trajectory_msgs::msg::JointTrajectory make_positions_chunk(
  const std::vector<std::vector<double>> & positions_per_point, double dt)
{
  trajectory_msgs::msg::JointTrajectory traj;
  for (size_t i = 0; i < positions_per_point.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions_per_point[i];
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * dt);
    traj.points.push_back(point);
  }
  return traj;
}

double knot_time(const trajectory_msgs::msg::JointTrajectory & traj, size_t i)
{
  return rclcpp::Duration(traj.points[i].time_from_start).seconds();
}

// Acceleration of the per-segment cubic Hermite evaluated at the knot, from
// either side: arrive = end of segment [i-1, i]; leave = start of [i, i+1].
double accel_arrive(const trajectory_msgs::msg::JointTrajectory & traj, size_t i, size_t joint)
{
  const double duration = knot_time(traj, i) - knot_time(traj, i - 1);
  const double pos_a = traj.points[i - 1].positions[joint];
  const double pos_b = traj.points[i].positions[joint];
  const double vel_a = traj.points[i - 1].velocities[joint];
  const double vel_b = traj.points[i].velocities[joint];
  return (6.0 * pos_a - 6.0 * pos_b) / (duration * duration) +
         (2.0 * vel_a + 4.0 * vel_b) / duration;
}

double accel_leave(const trajectory_msgs::msg::JointTrajectory & traj, size_t i, size_t joint)
{
  const double duration = knot_time(traj, i + 1) - knot_time(traj, i);
  const double pos_a = traj.points[i].positions[joint];
  const double pos_b = traj.points[i + 1].positions[joint];
  const double vel_a = traj.points[i].velocities[joint];
  const double vel_b = traj.points[i + 1].velocities[joint];
  return (-6.0 * pos_a + 6.0 * pos_b) / (duration * duration) -
         (4.0 * vel_a + 2.0 * vel_b) / duration;
}
}  // namespace

/**
 * @brief fill_cubic_spline_velocities makes the per-segment cubic Hermite C2:
 * acceleration is continuous across every interior knot, with rest boundary
 * conditions (v0 = v_{N-1} = 0), for each joint solved independently.
 */
TEST(TestTrajectory, fill_cubic_spline_velocities_makes_acceleration_continuous)
{
  const double dt = 0.1;
  std::vector<std::vector<double>> positions_per_point;
  for (int i = 0; i < 7; ++i)
  {
    // two joints with distinct paths
    positions_per_point.push_back({0.1 + 0.4 * std::sin(M_PI * i * dt), 0.5 - 0.1 * i});
  }
  auto traj = make_positions_chunk(positions_per_point, dt);

  joint_trajectory_controller::fill_cubic_spline_velocities(traj);

  const size_t n = traj.points.size();
  for (const auto & point : traj.points)
  {
    ASSERT_EQ(point.velocities.size(), 2u);
  }
  for (size_t joint = 0; joint < 2; ++joint)
  {
    for (size_t i = 1; i + 1 < n; ++i)
    {
      EXPECT_NEAR(accel_arrive(traj, i, joint), accel_leave(traj, i, joint), 1e-6)
        << "acceleration discontinuous at knot " << i << " joint " << joint;
    }
    EXPECT_NEAR(traj.points.front().velocities[joint], 0.0, 1e-9);
    EXPECT_NEAR(traj.points.back().velocities[joint], 0.0, 1e-9);
  }
}

/**
 * @brief End-to-end through the real sampler: a positions-only chunk that is
 * velocity-filled and then sampled by Trajectory::sample() produces smooth,
 * bounded acceleration; the SAME positions left unfilled interpolate linearly
 * (C0) and show far larger acceleration spikes at the knots.
 */
TEST(TestTrajectory, sample_after_fill_is_smooth_not_staircase)
{
  // Realistic action chunk: 50 waypoints at 25 Hz (~2 s) on a smooth path.
  const double dt = 1.0 / 25.0;
  const int n = 50;
  auto make_msg = [&]()
  {
    auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    msg->header.stamp = rclcpp::Time(0);
    for (int i = 0; i < n; ++i)
    {
      const double s = static_cast<double>(i) / (n - 1);
      // sin^2 (Hann-shaped): zero slope at s=0 and s=1, so the spline's rest
      // boundary condition (v0 = v_N = 0) is natural.
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {0.1 + 0.5 * std::sin(M_PI * s) * std::sin(M_PI * s)};
      point.time_from_start = rclcpp::Duration::from_seconds(i * dt);
      msg->points.push_back(point);
    }
    return msg;
  };

  // Sample densely across the chunk span and return the peak |second difference|
  // of the sampled position (the acceleration actually experienced).
  auto peak_sampled_acceleration = [&](std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point_before;
    point_before.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point_before.positions = {msg->points.front().positions[0]};
    const rclcpp::Time time_now = rclcpp::Clock().now();
    auto traj = joint_trajectory_controller::Trajectory(time_now, point_before, msg);

    const double sample_dt = 0.002;
    const double span = (n - 1) * dt;
    std::vector<double> positions;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    trajectory_msgs::msg::JointTrajectoryPoint out;
    for (double t = 0.0; t <= span; t += sample_dt)
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(t), DEFAULT_INTERPOLATION, out, start, end);
      positions.push_back(out.positions[0]);
    }
    double peak = 0.0;
    for (size_t k = 2; k < positions.size(); ++k)
    {
      const double accel =
        (positions[k] - 2.0 * positions[k - 1] + positions[k - 2]) / (sample_dt * sample_dt);
      peak = std::max(peak, std::abs(accel));
    }
    return peak;
  };

  auto filled = make_msg();
  joint_trajectory_controller::fill_cubic_spline_velocities(*filled);
  auto unfilled = make_msg();  // positions only -> JTC linear (C0)

  const double peak_filled = peak_sampled_acceleration(filled);
  const double peak_unfilled = peak_sampled_acceleration(unfilled);

  double analytic_peak = 0.0;
  for (size_t i = 0; i + 1 < static_cast<size_t>(n); ++i)
  {
    analytic_peak = std::max(analytic_peak, std::abs(accel_leave(*filled, i, 0)));
  }
  EXPECT_LT(peak_filled, 2.0 * analytic_peak + 1.0)
    << "sampled acceleration should track the spline's analytic acceleration";
  EXPECT_GT(peak_unfilled, 3.0 * peak_filled)
    << "unfilled positions-only samples to a C0 staircase with far larger accel spikes";
}

/**
 * @brief Edge cases: a two-point chunk is a valid single segment (rest-to-rest);
 * trajectories with < 2 points or inconsistent widths are a safe no-op; and
 * back-to-back chunks of different sizes both fill correctly.
 */
TEST(TestTrajectory, fill_cubic_spline_velocities_edge_cases)
{
  // N == 2: valid single segment, rest BC -> both velocities 0.
  {
    auto traj = make_positions_chunk({{0.0}, {0.5}}, 0.1);
    joint_trajectory_controller::fill_cubic_spline_velocities(traj);
    ASSERT_EQ(traj.points[0].velocities.size(), 1u);
    EXPECT_NEAR(traj.points[0].velocities[0], 0.0, EPS);
    EXPECT_NEAR(traj.points[1].velocities[0], 0.0, EPS);
  }
  // N == 1 and empty: no-op (velocities stay empty), must not crash.
  {
    auto one = make_positions_chunk({{0.3}}, 0.1);
    joint_trajectory_controller::fill_cubic_spline_velocities(one);
    EXPECT_TRUE(one.points[0].velocities.empty());

    trajectory_msgs::msg::JointTrajectory empty;
    joint_trajectory_controller::fill_cubic_spline_velocities(empty);
    EXPECT_TRUE(empty.points.empty());
  }
  // Varying sizes back-to-back: a small then a large chunk both fill correctly.
  {
    auto small = make_positions_chunk({{0.0}, {0.1}, {0.2}}, 0.1);
    auto large =
      make_positions_chunk({{0.0}, {0.1}, {0.2}, {0.3}, {0.2}, {0.1}, {0.0}, {0.1}, {0.2}}, 0.1);
    joint_trajectory_controller::fill_cubic_spline_velocities(small);
    joint_trajectory_controller::fill_cubic_spline_velocities(large);
    ASSERT_EQ(small.points.size(), 3u);
    ASSERT_EQ(large.points.size(), 9u);
    for (const auto & point : small.points) EXPECT_EQ(point.velocities.size(), 1u);
    for (const auto & point : large.points) EXPECT_EQ(point.velocities.size(), 1u);
    EXPECT_NEAR(small.points.front().velocities[0], 0.0, EPS);
    EXPECT_NEAR(large.points.back().velocities[0], 0.0, EPS);
  }
}
