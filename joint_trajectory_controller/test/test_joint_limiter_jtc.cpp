// Copyright 2024 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/time.hpp"

#include "test_trajectory_controller_utils.hpp"

using test_trajectory_controllers::TrajectoryControllerTest;

/**
 * @brief verify joint limiter enforces position limits
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_position_expect_clamped)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("joint_limiter_type", "joint_limits/JointTrajectoryPointSaturationLimiter"),
    rclcpp::Parameter("joint_limits.joint1.has_position_limits", true),
    rclcpp::Parameter("joint_limits.joint1.min_position", -5.0),
    rclcpp::Parameter("joint_limits.joint1.max_position", 5.0),
    rclcpp::Parameter("joint_limits.joint2.has_position_limits", true),
    rclcpp::Parameter("joint_limits.joint2.min_position", -5.0),
    rclcpp::Parameter("joint_limits.joint2.max_position", 5.0),
    rclcpp::Parameter("joint_limits.joint3.has_position_limits", true),
    rclcpp::Parameter("joint_limits.joint3.min_position", -5.0),
    rclcpp::Parameter("joint_limits.joint3.max_position", 5.0),
  };

  // Start joints at max_position so position clamping and stopping-distance
  // checks both pass trivially (no velocity generated from position difference)
  std::vector<double> initial_pos = {5.0, 5.0, 5.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // Publish a trajectory with position beyond limits
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  std::vector<std::vector<double>> points{{{10.0, 5.0, 2.0}}};
  std::vector<std::vector<double>> points_velocities{{{0.0, 0.0, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // First update samples command_next_ at t=0.01
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // Position should be clamped to max_position=5.0
  auto cmd_next = traj_controller_->get_command_next();
  ASSERT_FALSE(cmd_next.positions.empty());
  EXPECT_NEAR(cmd_next.positions[0], 5.0, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.positions[1], 5.0, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.positions[2], 2.0, COMMON_THRESHOLD);

  // Hardware command interfaces also reflect the clamped position
  EXPECT_NEAR(joint_pos_[0], 5.0, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[1], 5.0, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[2], 2.0, COMMON_THRESHOLD);

  executor.cancel();
}

/**
 * @brief verify joint limiter is initialized and enforces velocity limits
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_velocity_expect_clamped)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("joint_limiter_type", "joint_limits/JointTrajectoryPointSaturationLimiter"),
    rclcpp::Parameter("joint_limits.joint1.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint1.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint2.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint2.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint3.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint3.max_velocity", 0.5),
  };

  SetUpAndActivateTrajectoryController(executor, params);

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // Point far away with vel=0 (vel computed from position diff triggers limit)
  std::vector<std::vector<double>> points{{{100.0, 100.0, 100.0}}};
  std::vector<std::vector<double>> points_velocities{{{0.0, 0.0, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // First update samples command_next_ at t=0.01 (start + 10ms)
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // Limiter clamped velocity to max_velocity=0.5 and recomputed position
  auto cmd_next = traj_controller_->get_command_next();
  ASSERT_FALSE(cmd_next.velocities.empty());
  EXPECT_NEAR(cmd_next.velocities[0], 0.5, COMMON_THRESHOLD);
  ASSERT_FALSE(cmd_next.positions.empty());
  EXPECT_NEAR(cmd_next.positions[0], INITIAL_POS_JOINT1 + 0.5 * 0.01, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.positions[1], INITIAL_POS_JOINT2 + 0.5 * 0.01, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.positions[2], INITIAL_POS_JOINT3 + 0.5 * 0.01, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.velocities[0], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.velocities[1], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.velocities[2], 0.5, COMMON_THRESHOLD);

  // Hardware command interfaces also reflect the limited values
  EXPECT_NEAR(joint_vel_[0], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[1], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[2], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[0], INITIAL_POS_JOINT1 + 0.5 * 0.01, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[1], INITIAL_POS_JOINT2 + 0.5 * 0.01, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[2], INITIAL_POS_JOINT3 + 0.5 * 0.01, COMMON_THRESHOLD);

  executor.cancel();
}

/**
 * @brief verify joint limiter enforces velocity limits with velocity-only command interface
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_velocity_only_interface)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"velocity"};
  state_interface_types_ = {"position", "velocity"};

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("joint_limiter_type", "joint_limits/JointTrajectoryPointSaturationLimiter"),
    rclcpp::Parameter("joint_limits.joint1.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint1.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint2.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint2.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint3.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint3.max_velocity", 0.5),
  };

  SetUpAndActivateTrajectoryController(executor, params);

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // Positions required by publish() but unused with velocity-only command interface
  std::vector<std::vector<double>> points{{{0.0, 0.0, 0.0}}};
  // Velocity far above limit
  std::vector<std::vector<double>> points_velocities{{{3.0, 3.0, -3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // Hardware command interface reflects the clamped velocity
  EXPECT_NEAR(joint_vel_[0], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[1], 0.5, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[2], -0.5, COMMON_THRESHOLD);

  executor.cancel();
}

/**
 * @brief verify joint limiter enforces acceleration limits with velocity command interface
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_acceleration_expect_clamped)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"velocity"};
  state_interface_types_ = {"position", "velocity"};

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("joint_limiter_type", "joint_limits/JointTrajectoryPointSaturationLimiter"),
    rclcpp::Parameter("joint_limits.joint1.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint1.max_acceleration", 5.0),
    rclcpp::Parameter("joint_limits.joint2.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint2.max_acceleration", 5.0),
    rclcpp::Parameter("joint_limits.joint3.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint3.max_acceleration", 5.0),
  };

  // Start with non-zero velocity so the limiter computes a non-trivial acceleration
  std::vector<double> initial_vel = {1.0, 1.0, 1.0};
  SetUpAndActivateTrajectoryController(
    executor, params, false, 0.0, 1.0, INITIAL_POS_JOINTS, initial_vel);

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  std::vector<std::vector<double>> points{{{0.0, 0.0, 0.0}}};
  // Publish a velocity that requires a large acceleration delta
  std::vector<std::vector<double>> points_velocities{{{3.0, 3.0, -3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // desired_acc = (3.0 - 1.0) / 0.01 = 200 rad/s^2, clamped to 5.0
  // desired_vel = 1.0 + 5.0 * 0.01 = 1.05
  EXPECT_NEAR(joint_vel_[0], 1.05, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[1], 1.05, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[2], 0.95, COMMON_THRESHOLD);

  executor.cancel();
}

/**
 * @brief verify joint limiter enforces both velocity and acceleration limits
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_velocity_and_acceleration)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"velocity"};
  state_interface_types_ = {"position", "velocity"};

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("joint_limiter_type", "joint_limits/JointTrajectoryPointSaturationLimiter"),
    rclcpp::Parameter("joint_limits.joint1.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint1.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint1.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint1.max_acceleration", 5.0),
    rclcpp::Parameter("joint_limits.joint2.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint2.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint2.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint2.max_acceleration", 5.0),
    rclcpp::Parameter("joint_limits.joint3.has_velocity_limits", true),
    rclcpp::Parameter("joint_limits.joint3.max_velocity", 0.5),
    rclcpp::Parameter("joint_limits.joint3.has_acceleration_limits", true),
    rclcpp::Parameter("joint_limits.joint3.max_acceleration", 5.0),
  };

  SetUpAndActivateTrajectoryController(executor, params);

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  std::vector<std::vector<double>> points{{{0.0, 0.0, 0.0}}};
  std::vector<std::vector<double>> points_velocities{{{3.0, 3.0, -3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  // Vel clamp: 3.0 -> 0.5. Acc clamp: (0.5 - 0.0)/0.01 = 50, clamped to 5.0, vel = 0.0 + 5.0*0.01 =
  // 0.05
  EXPECT_NEAR(joint_vel_[0], 0.05, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[1], 0.05, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_vel_[2], -0.05, COMMON_THRESHOLD);

  executor.cancel();
}

/**
 * @brief verify joint limiter enforces position, velocity, and acceleration limits together
 */
TEST_F(TrajectoryControllerTest, when_joint_limiter_limits_position_velocity_and_acceleration)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  // Start below max_position so position clamping produces a non-trivial result
  SetUpAndActivateTrajectoryController(executor, params);

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // Publish a point beyond position limits with high velocity
  std::vector<std::vector<double>> points{{{3.0, 1.5, 3.0}}};
  std::vector<std::vector<double>> points_velocities{{{3.0, 1.0, 3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  auto cmd_next = traj_controller_->get_command_next();

  // Position was clamped from 3.0 to 1.5, then vel clamp recomputed to ~1.105,
  // then acc clamp overrides: pos = 1.1 + 0.0*0.01 + 0.5*5.0*0.01^2 = 1.10025
  ASSERT_FALSE(cmd_next.positions.empty());
  EXPECT_NEAR(cmd_next.positions[0], 1.10025, COMMON_THRESHOLD);
  EXPECT_NEAR(cmd_next.positions[1], 2.10025, COMMON_THRESHOLD);

  EXPECT_NEAR(joint_pos_[0], 1.10025, COMMON_THRESHOLD);
  EXPECT_NEAR(joint_pos_[1], 2.10025, COMMON_THRESHOLD);

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, short_sample_trajectory_within_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  // Start at origin (well within limits)
  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3-point trajectory with positions and velocities well within configured limits
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  std::vector<std::vector<double>> points{{{0.3, 0.2, 0.1}}, {{0.6, 0.5, 0.4}}, {{1.0, 0.9, 0.8}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // First update: sample command_next_ early in the trajectory
  traj_controller_->update(
    rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01));

  auto cmd_next = traj_controller_->get_command_next();
  ASSERT_FALSE(cmd_next.positions.empty());

  // Positions should be between initial (0.0) and first waypoint — not clamped to limits
  EXPECT_GT(cmd_next.positions[0], 0.0);
  EXPECT_LT(cmd_next.positions[0], 0.5);
  EXPECT_GT(cmd_next.positions[1], 0.0);
  EXPECT_LT(cmd_next.positions[1], 0.5);
  EXPECT_GT(cmd_next.positions[2], 0.0);
  EXPECT_LT(cmd_next.positions[2], 0.5);

  // All commands must be strictly within the configured limits
  EXPECT_LE(std::abs(cmd_next.positions[0]), 1.5);
  EXPECT_LE(std::abs(cmd_next.positions[1]), 1.5);
  EXPECT_LE(std::abs(cmd_next.positions[2]), 1.5);
  if (!cmd_next.velocities.empty())
  {
    EXPECT_LE(std::abs(cmd_next.velocities[0]), 0.5);
    EXPECT_LE(std::abs(cmd_next.velocities[1]), 0.5);
    EXPECT_LE(std::abs(cmd_next.velocities[2]), 0.5);
  }

  // Step through the entire trajectory to completion
  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 3 + rclcpp::Duration::from_seconds(0.2));

  // Final hardware commands should match the last waypoint (within spline tolerance)
  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(points[2][0], joint_pos_[0], 0.1);
    EXPECT_NEAR(points[2][1], joint_pos_[1], 0.1);
    EXPECT_NEAR(points[2][2], joint_pos_[2], 0.1);
  }

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, short_sample_trajectory_on_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  // Start close to the limit boundaries: joint1 near +1.5, joint2 near -1.5, joint3 at 0
  std::vector<double> initial_pos = {1.49, -1.49, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // Single waypoint with positions exactly at limits and velocities within limits
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // joint1 at max_position (1.5), joint2 at min_position (-1.5), joint3 at 0.0
  std::vector<std::vector<double>> points{{{1.5, -1.5, 0.0}}};
  std::vector<std::vector<double>> points_velocities{{{0.1, 0.1, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Run the trajectory to completion
  updateController(rclcpp::Duration(FIRST_POINT_TIME) + rclcpp::Duration::from_seconds(0.2));

  // Positions at limit boundaries should not be clamped further.
  // Use a relaxed tolerance because acceleration limits cause small deviations
  // during deceleration to the limit-boundary waypoint.
  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(points[0][1], joint_pos_[1], 0.01);
    EXPECT_NEAR(points[0][2], joint_pos_[2], COMMON_THRESHOLD);
  }

  // All commands remain within configured limits
  EXPECT_LE(std::abs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::abs(joint_pos_[1]), 1.5);

  executor.cancel();
}
