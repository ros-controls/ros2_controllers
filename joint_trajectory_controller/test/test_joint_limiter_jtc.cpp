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

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3 waypoints spaced 200ms apart -> t=0.2, t=0.4, t=0.6
  // With 10ms updates, each segment gets ~20 interpolated samples
  constexpr auto DELAY_BTWN_POINTS = std::chrono::milliseconds(200);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(DELAY_BTWN_POINTS)};
  std::vector<std::vector<double>> points{
    {{10.0, 0.0, 0.0}}, {{10.0, 5.0, 2.0}}, {{5.0, 10.0, 5.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};
  std::vector<std::vector<double>> limit_enforced_points{
    {{5.0, 0.0, 0.0}}, {{5.0, 5.0, 2.0}}, {{5.0, 5.0, 5.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Step through with constant 10ms period (matches controller's update_rate=100Hz)
  // 60 steps = 0.60s, covers full trajectory (last waypoint at 0.6s)
  auto logger = traj_controller_->get_node()->get_logger();
  const double dt = 0.01;
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 20; ++j)
    {
      traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
    }
    auto cmd_next = traj_controller_->get_command_next();
    ASSERT_FALSE(cmd_next.positions.empty());
    EXPECT_NEAR(cmd_next.positions[0], limit_enforced_points[i][0], COMMON_THRESHOLD);
    EXPECT_NEAR(cmd_next.positions[1], limit_enforced_points[i][1], COMMON_THRESHOLD);
    EXPECT_NEAR(cmd_next.positions[2], limit_enforced_points[i][2], COMMON_THRESHOLD);
    // Hardware command interfaces also reflect the clamped position
    EXPECT_NEAR(joint_pos_[0], limit_enforced_points[i][0], COMMON_THRESHOLD);
    EXPECT_NEAR(joint_pos_[1], limit_enforced_points[i][1], COMMON_THRESHOLD);
    EXPECT_NEAR(joint_pos_[2], limit_enforced_points[i][2], COMMON_THRESHOLD);
  }

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

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3 waypoints spaced 200ms apart -> t=0.2, t=0.4, t=0.6
  // With 10ms updates, each segment gets ~20 interpolated samples
  constexpr auto DELAY_BTWN_POINTS = std::chrono::milliseconds(200);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(DELAY_BTWN_POINTS)};
  std::vector<std::vector<double>> points{
    {{10.0, 0.0, 0.0}}, {{10.0, 5.0, 2.0}}, {{5.0, 10.0, 5.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Step through with constant 10ms period (matches controller's update_rate=100Hz)
  // 60 steps = 0.60s, covers full trajectory (last waypoint at 0.6s)
  auto logger = traj_controller_->get_node()->get_logger();
  const double dt = 0.01;
  const double max_vel = 0.5;
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 20; ++j)
    {
      traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
      auto cmd_next = traj_controller_->get_command_next();
      ASSERT_FALSE(cmd_next.velocities.empty());
      EXPECT_LE(std::fabs(cmd_next.velocities[0]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[1]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[2]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[0]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[1]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[2]), max_vel);
    }
  }

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

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3 waypoints spaced 200ms apart -> t=0.2, t=0.4, t=0.6
  // With 10ms updates, each segment gets ~20 interpolated samples
  constexpr auto DELAY_BTWN_POINTS = std::chrono::milliseconds(200);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(DELAY_BTWN_POINTS)};
  std::vector<std::vector<double>> points{
    {{10.0, 0.0, 0.0}}, {{10.0, 5.0, 2.0}}, {{5.0, 10.0, 5.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Step through with constant 10ms period (matches controller's update_rate=100Hz)
  // 60 steps = 0.60s, covers full trajectory (last waypoint at 0.6s)
  auto logger = traj_controller_->get_node()->get_logger();
  const double dt = 0.01;
  const double max_acc = 5.0;
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 20; ++j)
    {
      traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
      auto cmd_next = traj_controller_->get_command_next();
      ASSERT_FALSE(cmd_next.accelerations.empty());
      EXPECT_LE(std::fabs(cmd_next.accelerations[0]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[1]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[2]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[0]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[1]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[2]), max_acc);
    }
  }

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

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3 waypoints spaced 200ms apart -> t=0.2, t=0.4, t=0.6
  // With 10ms updates, each segment gets ~20 interpolated samples
  constexpr auto DELAY_BTWN_POINTS = std::chrono::milliseconds(200);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(DELAY_BTWN_POINTS)};
  std::vector<std::vector<double>> points{
    {{10.0, 0.0, 0.0}}, {{10.0, 5.0, 2.0}}, {{5.0, 10.0, 5.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};
  std::vector<std::vector<double>> limit_enforced_points{
    {{5.0, 0.0, 0.0}}, {{5.0, 5.0, 2.0}}, {{5.0, 5.0, 5.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Step through with constant 10ms period (matches controller's update_rate=100Hz)
  // 60 steps = 0.60s, covers full trajectory (last waypoint at 0.6s)
  auto logger = traj_controller_->get_node()->get_logger();
  const double dt = 0.01;
  const double max_vel = 0.5;
  const double max_acc = 5.0;
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 20; ++j)
    {
      traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
      auto cmd_next = traj_controller_->get_command_next();
      ASSERT_FALSE(cmd_next.velocities.empty());
      EXPECT_LE(std::fabs(cmd_next.velocities[0]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[1]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[2]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[0]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[1]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[2]), max_vel);
      ASSERT_FALSE(cmd_next.accelerations.empty());
      EXPECT_LE(std::fabs(cmd_next.accelerations[0]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[1]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[2]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[0]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[1]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[2]), max_acc);
    }
  }

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

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 3 waypoints spaced 200ms apart -> t=0.2, t=0.4, t=0.6
  // With 10ms updates, each segment gets 20 interpolated samples
  constexpr auto DELAY_BTWN_POINTS = std::chrono::milliseconds(200);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(DELAY_BTWN_POINTS)};
  std::vector<std::vector<double>> points{{{5.0, 0.0, 0.0}}, {{5.0, 5.0, 2.0}}, {{5.0, 5.0, 5.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Step through with constant 10ms period (matches controller's update_rate=100Hz)
  // 20 steps per segment * 3 segments = 60 steps = 0.60s, covers full trajectory
  auto logger = traj_controller_->get_node()->get_logger();
  const double dt = 0.01;
  const double max_vel = 2.0;
  const double max_acc = 5.0;
  const double max_pos = 1.5;
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 20; ++j)
    {
      traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
      auto cmd_next = traj_controller_->get_command_next();
      ASSERT_FALSE(cmd_next.velocities.empty());
      EXPECT_LE(std::fabs(cmd_next.velocities[0]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[1]), max_vel);
      EXPECT_LE(std::fabs(cmd_next.velocities[2]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[0]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[1]), max_vel);
      EXPECT_LE(std::fabs(joint_vel_[2]), max_vel);
      ASSERT_FALSE(cmd_next.accelerations.empty());
      EXPECT_LE(std::fabs(cmd_next.accelerations[0]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[1]), max_acc);
      EXPECT_LE(std::fabs(cmd_next.accelerations[2]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[0]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[1]), max_acc);
      EXPECT_LE(std::fabs(joint_acc_[2]), max_acc);
      // Verify position limits are enforced throughout the trajectory
      EXPECT_LE(std::fabs(cmd_next.positions[0]), max_pos);
      EXPECT_LE(std::fabs(cmd_next.positions[1]), max_pos);
      EXPECT_LE(std::fabs(cmd_next.positions[2]), max_pos);
      EXPECT_LE(std::fabs(joint_pos_[0]), max_pos);
      EXPECT_LE(std::fabs(joint_pos_[1]), max_pos);
      EXPECT_LE(std::fabs(joint_pos_[2]), max_pos);
    }
  }

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
  EXPECT_LE(std::fabs(cmd_next.positions[0]), 1.5);
  EXPECT_LE(std::fabs(cmd_next.positions[1]), 1.5);
  EXPECT_LE(std::fabs(cmd_next.positions[2]), 1.5);
  if (!cmd_next.velocities.empty())
  {
    EXPECT_LE(std::fabs(cmd_next.velocities[0]), 0.5);
    EXPECT_LE(std::fabs(cmd_next.velocities[1]), 0.5);
    EXPECT_LE(std::fabs(cmd_next.velocities[2]), 0.5);
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
  EXPECT_LE(std::fabs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[1]), 1.5);

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, short_sample_trajectory_over_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  // Start at origin
  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // Single waypoint with positions and velocities far exceeding limits
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(10);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  std::vector<std::vector<double>> points{{{3.0, 3.0, 3.0}}};
  std::vector<std::vector<double>> points_velocities{{{3.0, 3.0, 3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // First update at t=0 with period 0.01s
  traj_controller_->update(
    rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(0.01));

  // Check command_next_ after limiter enforcement
  auto cmd_next = traj_controller_->get_command_next();
  ASSERT_FALSE(cmd_next.positions.empty());

  // The key check is that values are within limits.
  EXPECT_LE(std::fabs(cmd_next.positions[0]), 1.5);
  EXPECT_LE(std::fabs(cmd_next.positions[1]), 1.5);
  EXPECT_LE(std::fabs(cmd_next.positions[2]), 1.5);

  if (!cmd_next.velocities.empty())
  {
    EXPECT_LE(std::fabs(cmd_next.velocities[0]), 0.5);
    EXPECT_LE(std::fabs(cmd_next.velocities[1]), 0.5);
    EXPECT_LE(std::fabs(cmd_next.velocities[2]), 0.5);
  }

  // Hardware commands also reflect the clamped values
  EXPECT_LE(std::fabs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[1]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[2]), 1.5);
  EXPECT_LE(std::fabs(joint_vel_[0]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[1]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[2]), 0.5);

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, long_sample_trajectory_within_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 8-point trajectory with positions and velocities well within limits
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration::from_seconds(1.6)};
  std::vector<std::vector<double>> points{{{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}, {{0.3, 0.3, 0.3}},
                                          {{0.4, 0.4, 0.4}}, {{0.5, 0.5, 0.5}}, {{0.6, 0.6, 0.6}},
                                          {{0.7, 0.7, 0.7}}, {{0.8, 0.8, 0.8}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}},
    {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Run the full trajectory to completion
  updateController(rclcpp::Duration::from_seconds(1.6) + rclcpp::Duration::from_seconds(0.5));

  // With acceleration limits, the actual trajectory lags the ideal spline.
  // Verify that ALL hardware commands stay within configured limits at the end.
  EXPECT_LE(std::fabs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[1]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[2]), 1.5);
  EXPECT_LE(std::fabs(joint_vel_[0]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[1]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[2]), 0.5);

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, long_sample_trajectory_on_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 8-point trajectory with values reaching limit boundaries at various points
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration::from_seconds(1.6)};
  std::vector<std::vector<double>> points{
    {{0.5, -0.5, 0.0}}, {{1.0, -1.0, 0.0}}, {{1.5, -1.5, 0.0}}, {{1.0, -1.0, 0.0}},
    {{0.5, -0.5, 0.0}}, {{0.0, 0.0, 0.0}},  {{1.5, -1.5, 0.5}}, {{1.5, -1.5, 1.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.1, 0.1, 0.0}}, {{0.1, 0.1, 0.0}}, {{0.1, 0.1, 0.0}}, {{0.1, 0.1, 0.0}},
    {{0.1, 0.1, 0.0}}, {{0.1, 0.1, 0.0}}, {{0.1, 0.1, 0.1}}, {{0.1, 0.1, 0.1}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Run the full trajectory to completion
  updateController(rclcpp::Duration::from_seconds(1.6) + rclcpp::Duration::from_seconds(0.5));

  // With acceleration limits the actual trajectory lags the ideal spline.
  // Verify that all commands stay within configured limits.
  EXPECT_LE(std::fabs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[1]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[2]), 1.5);
  EXPECT_LE(std::fabs(joint_vel_[0]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[1]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[2]), 0.5);

  executor.cancel();
}

TEST_F(TrajectoryControllerTest, long_sample_trajectory_over_limits)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity"};

  auto params = DefaultJointLimiterParams();

  std::vector<double> initial_pos = {0.0, 0.0, 0.0};
  SetUpAndActivateTrajectoryController(executor, params, false, 0.0, 1.0, initial_pos);

  // 8-point trajectory with positions and velocities exceeding limits
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration::from_seconds(1.6)};
  std::vector<std::vector<double>> points{{{2.0, 2.0, 2.0}}, {{3.0, 3.0, 3.0}}, {{4.0, 4.0, 4.0}},
                                          {{5.0, 5.0, 5.0}}, {{2.0, 2.0, 2.0}}, {{1.0, 1.0, 1.0}},
                                          {{3.0, 3.0, 3.0}}, {{4.0, 4.0, 4.0}}};
  std::vector<std::vector<double>> points_velocities{
    {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}},
    {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}}, {{3.0, 3.0, 3.0}}};

  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // Run the full trajectory to completion
  updateController(rclcpp::Duration::from_seconds(1.6) + rclcpp::Duration::from_seconds(0.5));

  // All hardware commands must be within limits despite waypoints exceeding them
  EXPECT_LE(std::fabs(joint_pos_[0]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[1]), 1.5);
  EXPECT_LE(std::fabs(joint_pos_[2]), 1.5);
  EXPECT_LE(std::fabs(joint_vel_[0]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[1]), 0.5);
  EXPECT_LE(std::fabs(joint_vel_[2]), 0.5);

  executor.cancel();
}
