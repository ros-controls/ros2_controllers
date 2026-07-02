// Copyright (c) 2026 ros2_control Development Team
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

// Unit tests for the bridge-specific policy logic (timing synthesis,
// positions-only gating, header-stamp preservation). The spline math itself
// lives in trajectory.cpp and is tested in test_trajectory.cpp.

#include <gmock/gmock.h>

#include <vector>

#include "joint_trajectory_controller/inference_bridge_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
// Exposes the bridge's protected policy helpers for unit testing; neither uses
// the controller node, so no init()/configure() is required.
class TestableBridge : public inference_bridge_controller::InferenceBridgeController
{
public:
  using InferenceBridgeController::is_positions_only;
  using InferenceBridgeController::synthesize_timing;
};

trajectory_msgs::msg::JointTrajectory make_positions_chunk(
  const std::vector<double> & positions, double dt)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1"};
  for (size_t i = 0; i < positions.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {positions[i]};
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * dt);
    traj.points.push_back(point);
  }
  return traj;
}

double time_at(const trajectory_msgs::msg::JointTrajectory & traj, size_t i)
{
  return rclcpp::Duration(traj.points[i].time_from_start).seconds();
}
}  // namespace

/**
 * @brief Untimed chunks get time_from_start = i / policy_frequency.
 */
TEST(TestInferenceBridgeController, synthesizes_timing_from_policy_frequency)
{
  TestableBridge bridge;  // default policy_frequency = 30 Hz
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1"};
  for (int i = 0; i < 5; ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.1 * i};
    traj.points.push_back(point);  // no time_from_start
  }

  bridge.synthesize_timing(traj);

  for (int i = 0; i < 5; ++i)
  {
    EXPECT_NEAR(time_at(traj, static_cast<size_t>(i)), i / 30.0, 1e-9);
  }
}

/**
 * @brief Timing the policy already set is left untouched.
 */
TEST(TestInferenceBridgeController, preserves_existing_timing)
{
  TestableBridge bridge;
  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.123);  // non-default spacing
  bridge.synthesize_timing(traj);
  EXPECT_NEAR(time_at(traj, 1), 0.123, 1e-9);  // unchanged
  EXPECT_NEAR(time_at(traj, 2), 0.246, 1e-9);
}

/**
 * @brief is_positions_only gates the upsampling: positions-only chunks are
 * detected, while a chunk that already carries velocities is passed through.
 */
TEST(TestInferenceBridgeController, detects_positions_only_and_passthrough)
{
  TestableBridge bridge;
  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.05);
  EXPECT_TRUE(bridge.is_positions_only(traj));

  traj.points[1].velocities = {0.5};  // derivatives present -> pass through
  EXPECT_FALSE(bridge.is_positions_only(traj));
}

/**
 * @brief The bridge transform (timing synthesis + velocity fill) leaves
 * header.stamp untouched, so JTC's immediate (stamp=0) vs deferred (future
 * stamp) install behaviour is unaffected by the bridge.
 */
TEST(TestInferenceBridgeController, preserves_header_stamp)
{
  TestableBridge bridge;
  auto traj = make_positions_chunk({0.0, 0.1, 0.2, 0.1, 0.0}, 0.05);
  // Clear timing so synthesize_timing actually runs, but keep a non-zero stamp.
  for (auto & point : traj.points)
  {
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  }
  traj.header.stamp.sec = 123;
  traj.header.stamp.nanosec = 456u;

  bridge.synthesize_timing(traj);
  joint_trajectory_controller::fill_cubic_spline_velocities(traj);

  EXPECT_EQ(traj.header.stamp.sec, 123);
  EXPECT_EQ(traj.header.stamp.nanosec, 456u);
}
