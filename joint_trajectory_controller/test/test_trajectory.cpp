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

#include "gmock/gmock.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
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

    trajectory_msgs::msg::JointTrajectoryPoint expected_point;
    joint_trajectory_controller::TrajectoryPointConstIter start, end;
    traj.sample(clock.now(), DEFAULT_INTERPOLATION, expected_point, start, end);

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
    ASSERT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
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
    double half_p2_to_p3 = (p2.positions[0] + p3.positions[0]) * 0.5;
    EXPECT_NEAR(half_p2_to_p3, expected_state.positions[0], EPS);
  }

  // sample 3s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.0), DEFAULT_INTERPOLATION, expected_state, start,
      end);
    EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
  }

  // sample past given points
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(3.125), DEFAULT_INTERPOLATION, expected_state,
      start, end);
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
    EXPECT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
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
    EXPECT_EQ(result, false);
  }

  // sample 0.5s after msg
  {
    traj.sample(
      time_now + rclcpp::Duration::from_seconds(0.5), DEFAULT_INTERPOLATION, expected_state, start,
      end);
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
    EXPECT_EQ((--traj.end()), start);
    EXPECT_EQ(traj.end(), end);
    EXPECT_NEAR(position_third_seg, expected_state.positions[0], EPS);
    EXPECT_NEAR(velocity_third_seg, expected_state.velocities[0], EPS);
    EXPECT_NEAR(p3.accelerations[0], expected_state.accelerations[0], EPS);
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
      ASSERT_EQ(result, false);
    }

    // sample 0.5s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(0.5), no_interpolation, expected_state, start,
        end);
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
      ASSERT_EQ(traj.begin(), start);
      ASSERT_EQ((++traj.begin()), end);
      EXPECT_NEAR(p2.positions[0], expected_state.positions[0], EPS);
    }

    // sample 2.5s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(2.5), no_interpolation, expected_state, start,
        end);
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }

    // sample 3s after msg
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(3.0), no_interpolation, expected_state, start,
        end);
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }

    // sample past given points
    {
      traj.sample(
        time_now + rclcpp::Duration::from_seconds(3.125), no_interpolation, expected_state, start,
        end);
      ASSERT_EQ((--traj.end()), start);
      ASSERT_EQ(traj.end(), end);
      EXPECT_NEAR(p3.positions[0], expected_state.positions[0], EPS);
    }
  }
}
