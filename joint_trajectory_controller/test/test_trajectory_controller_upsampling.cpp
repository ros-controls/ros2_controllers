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

#include <gmock/gmock.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "test_trajectory_controller_utils.hpp"

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using test_trajectory_controllers::TrajectoryControllerTest;

namespace
{
// Exposes JTC's protected upsampling helpers and lets a test set the
// positions_upsampling.* parameters directly, without a node or lifecycle.
class UpsamplingHelpers : public joint_trajectory_controller::JointTrajectoryController
{
public:
  using JointTrajectoryController::is_positions_only;
  using JointTrajectoryController::prepend_commanded_state;
  using JointTrajectoryController::preprocess_incoming_trajectory;
  using JointTrajectoryController::synthesize_timing;

  void configure_upsampling(bool enabled, double policy_frequency)
  {
    params_.positions_upsampling.enable = enabled;
    params_.positions_upsampling.policy_frequency = policy_frequency;
  }

  // Stand in for an active controller that has already commanded a state.
  void configure_commanded_state(
    const std::vector<std::string> & joints, const std::vector<double> & positions,
    const std::vector<double> & velocities)
  {
    params_.joints = joints;
    dof_ = joints.size();
    trajectory_msgs::msg::JointTrajectoryPoint commanded;
    commanded.positions = positions;
    commanded.velocities = velocities;
    rt_last_commanded_state_.set(commanded);
  }
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

// The start velocity follows the sender's joint order: the message is not sorted until install.
trajectory_msgs::msg::JointTrajectory anchorable_chunk(const std::vector<std::string> & joints)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joints;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(joints.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(0.1);
  traj.points.push_back(point);
  return traj;
}

// a mismatch between the two would pair a joint with another joint's state
TEST(JtcUpsamplingHelpers, anchor_follows_the_message_joint_order)
{
  UpsamplingHelpers c;
  c.configure_commanded_state({"joint1", "joint2", "joint3"}, {1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});

  auto traj = anchorable_chunk({"joint3", "joint1", "joint2"});  // reordered
  std::vector<double> start_velocity;
  ASSERT_TRUE(c.prepend_commanded_state(traj, start_velocity));

  ASSERT_EQ(start_velocity.size(), 3u);
  EXPECT_NEAR(start_velocity[0], 0.3, 1e-12);
  EXPECT_NEAR(start_velocity[1], 0.1, 1e-12);
  EXPECT_NEAR(start_velocity[2], 0.2, 1e-12);

  ASSERT_EQ(traj.points.size(), 2u);
  EXPECT_DOUBLE_EQ(time_at(traj, 0), 0.0);
  ASSERT_EQ(traj.points[0].positions.size(), 3u);
  EXPECT_NEAR(traj.points[0].positions[0], 3.0, 1e-12);
  EXPECT_NEAR(traj.points[0].positions[1], 1.0, 1e-12);
  EXPECT_NEAR(traj.points[0].positions[2], 2.0, 1e-12);
}

TEST(JtcUpsamplingHelpers, anchor_skipped_when_commanded_state_is_not_finite)
{
  UpsamplingHelpers c;
  c.configure_commanded_state(
    {"joint1", "joint2"}, {1.0, 2.0}, {0.1, std::numeric_limits<double>::quiet_NaN()});

  auto traj = anchorable_chunk({"joint1", "joint2"});
  std::vector<double> start_velocity;
  EXPECT_FALSE(c.prepend_commanded_state(traj, start_velocity));
  EXPECT_EQ(traj.points.size(), 1u);
}

TEST(JtcUpsamplingHelpers, anchor_skipped_when_the_stamp_is_not_zero)
{
  UpsamplingHelpers c;
  c.configure_commanded_state({"joint1"}, {1.0}, {0.1});

  auto traj = anchorable_chunk({"joint1"});
  traj.header.stamp = rclcpp::Time(5, 0, RCL_ROS_TIME);
  std::vector<double> start_velocity;
  EXPECT_FALSE(c.prepend_commanded_state(traj, start_velocity));
  EXPECT_EQ(traj.points.size(), 1u);
}

TEST(JtcUpsamplingHelpers, anchor_skipped_when_the_first_waypoint_is_at_zero)
{
  UpsamplingHelpers c;
  c.configure_commanded_state({"joint1"}, {1.0}, {0.1});

  auto traj = anchorable_chunk({"joint1"});
  traj.points[0].time_from_start = rclcpp::Duration(0, 0);
  std::vector<double> start_velocity;
  EXPECT_FALSE(c.prepend_commanded_state(traj, start_velocity));
  EXPECT_EQ(traj.points.size(), 1u);
}

// Untimed chunks get time_from_start = (i + 1) / policy_frequency.
TEST(JtcUpsamplingHelpers, synthesizes_timing_from_policy_frequency)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 30.0);

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1"};
  for (int i = 0; i < 5; ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.1 * i};
    traj.points.push_back(point);  // no time_from_start
  }

  c.synthesize_timing(traj);

  // the first waypoint is one policy step ahead, leaving [start, dt) for the lead-in
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_NEAR(time_at(traj, static_cast<size_t>(i)), (i + 1) / 30.0, 1e-9);
  }
}

// Timing the policy already set is left untouched.
TEST(JtcUpsamplingHelpers, preserves_existing_timing)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 30.0);

  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.123);  // non-default spacing
  c.synthesize_timing(traj);
  EXPECT_NEAR(time_at(traj, 1), 0.123, 1e-9);  // unchanged
  EXPECT_NEAR(time_at(traj, 2), 0.246, 1e-9);
}

// is_positions_only gates the upsampling: positions-only detected, velocity-carrying
// chunk passed through.
TEST(JtcUpsamplingHelpers, detects_positions_only_and_passthrough)
{
  UpsamplingHelpers c;
  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.05);
  EXPECT_TRUE(c.is_positions_only(traj));

  traj.points[1].velocities = {0.5};  // derivatives present -> pass through
  EXPECT_FALSE(c.is_positions_only(traj));
}

// A chunk carrying accelerations (but no velocities) is NOT positions-only: layering
// spline velocities on top would force an inconsistent quintic interpolation.
TEST(JtcUpsamplingHelpers, acceleration_carrying_chunk_is_not_positions_only)
{
  UpsamplingHelpers c;
  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.05);
  traj.points[1].accelerations = {0.5};
  EXPECT_FALSE(c.is_positions_only(traj));
}

// Feature off: preprocess is a strict no-op (JTC behaves exactly as before).
TEST(JtcUpsamplingHelpers, preprocess_is_noop_when_disabled)
{
  UpsamplingHelpers c;
  c.configure_upsampling(false, 30.0);

  auto traj = make_positions_chunk({0.0, 0.1, 0.2, 0.1, 0.0}, 0.0);  // untimed
  c.preprocess_incoming_trajectory(traj);

  for (const auto & point : traj.points)
  {
    EXPECT_TRUE(point.velocities.empty());
    EXPECT_EQ(rclcpp::Duration(point.time_from_start).seconds(), 0.0);
  }
}

// Feature on: an untimed positions-only chunk is timed and velocity-filled in place.
TEST(JtcUpsamplingHelpers, preprocess_upsamples_when_enabled)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 30.0);

  auto traj = make_positions_chunk({0.0, 0.1, 0.2, 0.1, 0.0}, 0.0);  // untimed
  c.preprocess_incoming_trajectory(traj);

  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    ASSERT_EQ(traj.points[i].velocities.size(), 1u);
    EXPECT_NEAR(time_at(traj, i), static_cast<double>(i + 1) / 30.0, 1e-9);
  }
  // rest boundary conditions
  EXPECT_NEAR(traj.points.front().velocities[0], 0.0, 1e-9);
  EXPECT_NEAR(traj.points.back().velocities[0], 0.0, 1e-9);
}

// Feature on but the chunk already carries velocities: passed through unchanged.
// A planner such as MoveIt sends the current state as its own first waypoint at t=0, so there is no
// room for an anchor and the point count the client sent must survive.
TEST(JtcUpsamplingHelpers, moveit_shaped_chunk_is_upsampled_without_an_anchor)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 0.0);  // chunks carry their own timing
  c.configure_commanded_state({"joint1"}, {0.0}, {0.5});

  auto traj = make_positions_chunk({0.0, 0.3, 0.6}, 0.5);  // first waypoint already at t=0
  const size_t sent = traj.points.size();
  c.preprocess_incoming_trajectory(traj);

  ASSERT_EQ(traj.points.size(), sent);
  EXPECT_DOUBLE_EQ(time_at(traj, 0), 0.0);
  EXPECT_DOUBLE_EQ(time_at(traj, sent - 1), 1.0);
  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    ASSERT_EQ(traj.points[i].velocities.size(), 1u);
  }
  EXPECT_NEAR(traj.points.front().velocities[0], 0.0, 1e-9);
  EXPECT_NEAR(traj.points.back().velocities[0], 0.0, 1e-9);
}

TEST(JtcUpsamplingHelpers, preprocess_passes_through_velocity_carrying_chunk)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 30.0);

  auto traj = make_positions_chunk({0.0, 0.1, 0.2}, 0.05);
  traj.points[0].velocities = {0.0};
  traj.points[1].velocities = {0.5};
  traj.points[2].velocities = {0.0};

  c.preprocess_incoming_trajectory(traj);
  EXPECT_NEAR(traj.points[1].velocities[0], 0.5, 1e-9);
}

// The upsampling transform leaves header.stamp untouched, so JTC's immediate
// (stamp=0) vs deferred (future stamp) install behaviour is unaffected.
TEST(JtcUpsamplingHelpers, preserves_header_stamp)
{
  UpsamplingHelpers c;
  c.configure_upsampling(true, 30.0);

  auto traj = make_positions_chunk({0.0, 0.1, 0.2, 0.1, 0.0}, 0.0);  // untimed
  traj.header.stamp.sec = 123;
  traj.header.stamp.nanosec = 456u;

  c.preprocess_incoming_trajectory(traj);

  EXPECT_EQ(traj.header.stamp.sec, 123);
  EXPECT_EQ(traj.header.stamp.nanosec, 456u);
}

// With upsampling off (default), an untimed positions-only chunk is rejected by
// validate_trajectory_msg (all-zero, non-increasing timestamps), so nothing installs.
TEST_F(TrajectoryControllerTest, upsampling_disabled_rejects_untimed_positions_only)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  const builtin_interfaces::msg::Duration zero_delay;  // -> all time_from_start = 0
  const std::vector<std::vector<double>> points{
    {{0.0, 0.0, 0.0}}, {{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}, {{0.1, 0.1, 0.1}}, {{0.0, 0.0, 0.0}}};
  publish(zero_delay, points, rclcpp::Time(0, 0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);
  updateController(rclcpp::Duration::from_seconds(0.05));

  EXPECT_FALSE(traj_controller_->has_nontrivial_traj());  // rejected -> still holding
  executor.cancel();
}

// With upsampling on, preprocess synthesizes timing so the same untimed chunk now
// passes validation and installs, and the reference carries (cubic) velocities. A second
// chunk arriving mid-execution must still blend into the first.
TEST_F(TrajectoryControllerTest, upsampling_enabled_accepts_untimed_positions_only)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(
    executor, {rclcpp::Parameter("positions_upsampling.enable", true),
               rclcpp::Parameter("positions_upsampling.policy_frequency", 30.0)});

  const builtin_interfaces::msg::Duration zero_delay;  // -> untimed chunk
  const std::vector<std::vector<double>> points{
    {{0.0, 0.0, 0.0}}, {{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}, {{0.1, 0.1, 0.1}}, {{0.0, 0.0, 0.0}}};
  publish(zero_delay, points, rclcpp::Time(0, 0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);
  updateController(rclcpp::Duration::from_seconds(0.05));

  EXPECT_TRUE(traj_controller_->has_nontrivial_traj());  // synthesized timing -> installed
  const auto state_reference = traj_controller_->get_state_reference();
  EXPECT_EQ(state_reference.velocities.size(), joint_names_.size());

  // a second chunk gets the anchor prepended: one point more than published, at t=0
  publish(zero_delay, points, rclcpp::Time(0, 0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);
  updateController(rclcpp::Duration::from_seconds(0.02));
  const auto installed = traj_controller_->get_installed_trajectory_msg();
  ASSERT_NE(installed, nullptr);
  EXPECT_EQ(installed->points.size(), points.size() + 1);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(installed->points.front().time_from_start).seconds(), 0.0);
  executor.cancel();
}
