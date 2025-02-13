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

#include <stddef.h>

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "test_trajectory_controller_utils.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;

TEST_P(TrajectoryControllerTestParameterized, configure_state_ignores_commands)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);
  traj_controller_->get_node()->set_parameter(
    rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", true));

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.5 * 1e9)), rclcpp::Duration::from_seconds(0.5));

  // hw position == 0 because controller is not activated
  EXPECT_EQ(0.0, joint_pos_[0]);
  EXPECT_EQ(0.0, joint_pos_[1]);
  EXPECT_EQ(0.0, joint_pos_[2]);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  compare_joints(joint_names_, joint_names_);
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names_with_command_joints)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // set command_joints parameter different than joint_names_
  const rclcpp::Parameter command_joint_names_param("command_joints", command_joint_names_);
  SetUpTrajectoryController(executor, {command_joint_names_param});

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  compare_joints(joint_names_, command_joint_names_);
}

TEST_P(TrajectoryControllerTestParameterized, activate)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  auto cmd_if_conf = traj_controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), joint_names_.size() * command_interface_types_.size());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = traj_controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_names_.size() * state_interface_types_.size());
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  state = ActivateTrajectoryController();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, cleanup)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(executor, params);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.5 * 1e9)), rclcpp::Duration::from_seconds(0.5));

  auto state = traj_controller_->get_node()->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  state = traj_controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, cleanup_after_configure)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  // configure controller
  auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  // cleanup controller
  state = traj_controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, correct_initialization_using_parameters)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);
  traj_controller_->get_node()->set_parameter(
    rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", true));

  // This call is replacing the way parameters are set via launch
  auto state = traj_controller_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  state = ActivateTrajectoryController();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());
  EXPECT_EQ(INITIAL_POS_JOINT1, joint_pos_[0]);
  EXPECT_EQ(INITIAL_POS_JOINT2, joint_pos_[1]);
  EXPECT_EQ(INITIAL_POS_JOINT3, joint_pos_[2]);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // first update
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));

  // wait for reaching the first point
  // controller would process the second point when deactivated below
  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.25 * 1e9)), rclcpp::Duration::from_seconds(0.15));
  EXPECT_TRUE(traj_controller_->has_active_traj());
  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(points.at(0).at(0), joint_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points.at(0).at(1), joint_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points.at(0).at(2), joint_pos_[2], COMMON_THRESHOLD);
  }

  // deactivate
  std::vector<double> deactivated_positions{joint_pos_[0], joint_pos_[1], joint_pos_[2]};
  state = traj_controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  // it should be holding the current point
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  expectHoldingPointDeactivated(deactivated_positions);

  // reactivate
  // wait so controller would have processed the third point when reactivated -> but it shouldn't
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  state = ActivateTrajectoryController(false, deactivated_positions);
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);

  // it should still be holding the position at time of deactivation
  // i.e., active but trivial trajectory (one point only)
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  expectCommandPoint(deactivated_positions);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, state_topic_legacy_consistency)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});
  subscribeToStateLegacy();
  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getStateLegacy();

  size_t n_joints = joint_names_.size();

  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names_[i], state->joint_names[i]);
  }

  // No trajectory by default, no desired state or error
  EXPECT_TRUE(state->desired.positions.empty() || state->desired.positions == INITIAL_POS_JOINTS);
  EXPECT_TRUE(state->desired.velocities.empty() || state->desired.velocities == INITIAL_VEL_JOINTS);
  EXPECT_TRUE(
    state->desired.accelerations.empty() || state->desired.accelerations == INITIAL_EFF_JOINTS);

  EXPECT_EQ(n_joints, state->actual.positions.size());
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "velocity") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->actual.velocities.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->actual.velocities.size());
  }
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "acceleration") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->actual.accelerations.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->actual.accelerations.size());
  }

  std::vector<double> zeros(3, 0);
  EXPECT_EQ(state->error.positions, zeros);
  EXPECT_TRUE(state->error.velocities.empty() || state->error.velocities == zeros);
  EXPECT_TRUE(state->error.accelerations.empty() || state->error.accelerations == zeros);
}

/**
 * @brief test if correct topic is received
 *
 * this test doesn't use class variables but subscribes to the state topic
 */
TEST_P(TrajectoryControllerTestParameterized, state_topic_consistency)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});
  subscribeToState(executor);
  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();

  size_t n_joints = joint_names_.size();

  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names_[i], state->joint_names[i]);
  }

  // No trajectory by default, no reference state or error
  EXPECT_TRUE(
    state->reference.positions.empty() || state->reference.positions == INITIAL_POS_JOINTS);
  EXPECT_TRUE(
    state->reference.velocities.empty() || state->reference.velocities == INITIAL_VEL_JOINTS);
  EXPECT_TRUE(
    state->reference.accelerations.empty() || state->reference.accelerations == INITIAL_EFF_JOINTS);

  std::vector<double> zeros(3, 0);
  EXPECT_EQ(state->error.positions, zeros);
  EXPECT_TRUE(state->error.velocities.empty() || state->error.velocities == zeros);
  EXPECT_TRUE(state->error.accelerations.empty() || state->error.accelerations == zeros);

  // expect feedback including all state_interfaces
  EXPECT_EQ(n_joints, state->feedback.positions.size());
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "velocity") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->feedback.velocities.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->feedback.velocities.size());
  }
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "acceleration") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->feedback.accelerations.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->feedback.accelerations.size());
  }

  // expect output including all command_interfaces
  if (
    std::find(command_interface_types_.begin(), command_interface_types_.end(), "position") ==
    command_interface_types_.end())
  {
    EXPECT_TRUE(state->output.positions.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->output.positions.size());
  }
  if (
    std::find(command_interface_types_.begin(), command_interface_types_.end(), "velocity") ==
    command_interface_types_.end())
  {
    EXPECT_TRUE(state->output.velocities.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->output.velocities.size());
  }
  if (
    std::find(command_interface_types_.begin(), command_interface_types_.end(), "acceleration") ==
    command_interface_types_.end())
  {
    EXPECT_TRUE(state->output.accelerations.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->output.accelerations.size());
  }
  if (
    std::find(command_interface_types_.begin(), command_interface_types_.end(), "effort") ==
    command_interface_types_.end())
  {
    EXPECT_TRUE(state->output.effort.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->output.effort.size());
  }
}

/**
 * @brief check if dynamic parameters are updated
 */
TEST_P(TrajectoryControllerTestParameterized, update_dynamic_parameters)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  SetUpAndActivateTrajectoryController(executor);

  updateControllerAsync();
  auto pids = traj_controller_->get_pids();

  if (traj_controller_->use_closed_loop_pid_adapter())
  {
    EXPECT_EQ(pids.size(), 3);
    auto gain_0 = pids.at(0)->get_gains();
    EXPECT_EQ(gain_0.p_gain_, 0.0);

    double kp = 1.0;
    SetPidParameters(kp);
    updateControllerAsync();

    pids = traj_controller_->get_pids();
    EXPECT_EQ(pids.size(), 3);
    gain_0 = pids.at(0)->get_gains();
    EXPECT_EQ(gain_0.p_gain_, kp);
  }
  else
  {
    // nothing to check here, skip further test
    EXPECT_EQ(pids.size(), 0);
  }

  executor.cancel();
}

/**
 * @brief check if dynamic tolerances are updated
 */
TEST_P(TrajectoryControllerTestParameterized, update_dynamic_tolerances)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  SetUpAndActivateTrajectoryController(executor);

  updateControllerAsync();

  // test default parameters
  {
    auto tols = traj_controller_->get_tolerances();
    EXPECT_EQ(tols.goal_time_tolerance, 0.0);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      EXPECT_EQ(tols.state_tolerance.at(i).position, 0.0);
      EXPECT_EQ(tols.goal_state_tolerance.at(i).position, 0.0);
      EXPECT_EQ(tols.goal_state_tolerance.at(i).velocity, 0.01);
    }
  }

  // change parameters, update and see what happens
  std::vector<rclcpp::Parameter> new_tolerances{
    rclcpp::Parameter("constraints.goal_time", 1.0),
    rclcpp::Parameter("constraints.stopped_velocity_tolerance", 0.02),
    rclcpp::Parameter("constraints.joint1.trajectory", 1.0),
    rclcpp::Parameter("constraints.joint2.trajectory", 2.0),
    rclcpp::Parameter("constraints.joint3.trajectory", 3.0),
    rclcpp::Parameter("constraints.joint1.goal", 10.0),
    rclcpp::Parameter("constraints.joint2.goal", 20.0),
    rclcpp::Parameter("constraints.joint3.goal", 30.0)};
  for (const auto & param : new_tolerances)
  {
    traj_controller_->get_node()->set_parameter(param);
  }
  updateControllerAsync();

  {
    auto tols = traj_controller_->get_tolerances();
    EXPECT_EQ(tols.goal_time_tolerance, 1.0);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      EXPECT_EQ(tols.state_tolerance.at(i).position, static_cast<double>(i) + 1.0);
      EXPECT_EQ(tols.goal_state_tolerance.at(i).position, 10.0 * (static_cast<double>(i) + 1.0));
      EXPECT_EQ(tols.goal_state_tolerance.at(i).velocity, 0.02);
    }
  }

  executor.cancel();
}

/**
 * @brief check if hold on startup
 */
TEST_P(TrajectoryControllerTestParameterized, hold_on_startup)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  SetUpAndActivateTrajectoryController(executor, {});

  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));
  // after startup, we expect an active trajectory:
  ASSERT_TRUE(traj_controller_->has_active_traj());
  // one point, being the position at startup
  std::vector<double> initial_positions{INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
  expectCommandPoint(initial_positions);

  executor.cancel();
}

// Floating-point value comparison threshold
const double EPS = 1e-6;
/**
 * @brief check if position error of revolute joints are wrapped around if not configured so
 */
TEST_P(TrajectoryControllerTestParameterized, position_error_not_angle_wraparound)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  constexpr double k_p = 10.0;
  std::vector<rclcpp::Parameter> params = {};
  bool angle_wraparound = false;
  SetUpAndActivateTrajectoryController(executor, params, true, k_p, 0.0, angle_wraparound);
  subscribeToState(executor);

  size_t n_joints = joint_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time());
  traj_controller_->wait_for_trajectory(executor);

  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // no update of state_interface
  EXPECT_EQ(state_feedback.positions, INITIAL_POS_JOINTS);

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_joints, state_reference.positions.size());
  EXPECT_EQ(n_joints, state_feedback.positions.size());
  EXPECT_EQ(n_joints, state_error.positions.size());

  // are the correct reference positions used?
  EXPECT_NEAR(points[0][0], state_reference.positions[0], COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][1], state_reference.positions[1], COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][2], state_reference.positions[2], COMMON_THRESHOLD);

  // no normalization of position error
  EXPECT_NEAR(state_error.positions[0], state_reference.positions[0] - INITIAL_POS_JOINTS[0], EPS);
  EXPECT_NEAR(state_error.positions[1], state_reference.positions[1] - INITIAL_POS_JOINTS[1], EPS);
  EXPECT_NEAR(state_error.positions[2], state_reference.positions[2] - INITIAL_POS_JOINTS[2], EPS);

  if (traj_controller_->has_position_command_interface())
  {
    // check command interface
    EXPECT_NEAR(points[0][0], joint_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][1], joint_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][2], joint_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // use_closed_loop_pid_adapter_
    if (traj_controller_->use_closed_loop_pid_adapter())
    {
      // we expect u = k_p * (s_d-s) for positions
      EXPECT_NEAR(
        k_p * (state_reference.positions[0] - INITIAL_POS_JOINTS[0]), joint_vel_[0],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference.positions[1] - INITIAL_POS_JOINTS[1]), joint_vel_[1],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference.positions[2] - INITIAL_POS_JOINTS[2]), joint_vel_[2],
        k_p * COMMON_THRESHOLD);
    }
    else
    {
      // interpolated points_velocities only
      // check command interface
      EXPECT_LT(0.0, joint_vel_[0]);
      EXPECT_LT(0.0, joint_vel_[1]);
      EXPECT_LT(0.0, joint_vel_[2]);
    }
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // with effort command interface, use_closed_loop_pid_adapter is always true
    // we expect u = k_p * (s_d-s) for positions
    EXPECT_NEAR(
      k_p * (state_reference.positions[0] - INITIAL_POS_JOINTS[0]), joint_eff_[0],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference.positions[1] - INITIAL_POS_JOINTS[1]), joint_eff_[1],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference.positions[2] - INITIAL_POS_JOINTS[2]), joint_eff_[2],
      k_p * COMMON_THRESHOLD);
  }

  executor.cancel();
}

/**
 * @brief check if position error of revolute joints are wrapped around if configured so
 */
TEST_P(TrajectoryControllerTestParameterized, position_error_angle_wraparound)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  constexpr double k_p = 10.0;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(executor, params, true, k_p, 0.0, true);

  size_t n_joints = joint_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // no update of state_interface
  EXPECT_EQ(state_feedback.positions, INITIAL_POS_JOINTS);

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_joints, state_reference.positions.size());
  EXPECT_EQ(n_joints, state_feedback.positions.size());
  EXPECT_EQ(n_joints, state_error.positions.size());

  // are the correct reference positions used?
  EXPECT_NEAR(points[0][0], state_reference.positions[0], COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][1], state_reference.positions[1], COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][2], state_reference.positions[2], COMMON_THRESHOLD);

  // is error.positions[2] wrapped around?
  EXPECT_NEAR(state_error.positions[0], state_reference.positions[0] - INITIAL_POS_JOINTS[0], EPS);
  EXPECT_NEAR(state_error.positions[1], state_reference.positions[1] - INITIAL_POS_JOINTS[1], EPS);
  EXPECT_NEAR(
    state_error.positions[2], state_reference.positions[2] - INITIAL_POS_JOINTS[2] - 2 * M_PI, EPS);

  if (traj_controller_->has_position_command_interface())
  {
    // check command interface
    EXPECT_NEAR(points[0][0], joint_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][1], joint_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][2], joint_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // use_closed_loop_pid_adapter_
    if (traj_controller_->use_closed_loop_pid_adapter())
    {
      // we expect u = k_p * (s_d-s) for joint0 and joint1
      EXPECT_NEAR(
        k_p * (state_reference.positions[0] - INITIAL_POS_JOINTS[0]), joint_vel_[0],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference.positions[1] - INITIAL_POS_JOINTS[1]), joint_vel_[1],
        k_p * COMMON_THRESHOLD);
      // is error of positions[2] wrapped around?
      EXPECT_GT(0.0, joint_vel_[2]);  // direction change because of angle wrap
      EXPECT_NEAR(
        k_p * (state_reference.positions[2] - INITIAL_POS_JOINTS[2] - 2 * M_PI), joint_vel_[2],
        k_p * COMMON_THRESHOLD);
    }
    else
    {
      // interpolated points_velocities only
      // check command interface
      EXPECT_LT(0.0, joint_vel_[0]);
      EXPECT_LT(0.0, joint_vel_[1]);
      EXPECT_LT(0.0, joint_vel_[2]);
    }
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // with effort command interface, use_closed_loop_pid_adapter is always true
    // we expect u = k_p * (s_d-s) for joint0 and joint1
    EXPECT_NEAR(
      k_p * (state_reference.positions[0] - INITIAL_POS_JOINTS[0]), joint_eff_[0],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference.positions[1] - INITIAL_POS_JOINTS[1]), joint_eff_[1],
      k_p * COMMON_THRESHOLD);
    // is error of positions[2] wrapped around?
    EXPECT_GT(0.0, joint_eff_[2]);
    EXPECT_NEAR(
      k_p * (state_reference.positions[2] - INITIAL_POS_JOINTS[2] - 2 * M_PI), joint_eff_[2],
      k_p * COMMON_THRESHOLD);
  }

  executor.cancel();
}

/**
 * @brief cmd_timeout must be greater than constraints.goal_time
 */
TEST_P(TrajectoryControllerTestParameterized, accept_correct_cmd_timeout)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // zero is default value, just for demonstration
  double cmd_timeout = 3.0;
  rclcpp::Parameter cmd_timeout_parameter("cmd_timeout", cmd_timeout);
  rclcpp::Parameter goal_time_parameter("constraints.goal_time", 2.0);
  SetUpAndActivateTrajectoryController(
    executor, {cmd_timeout_parameter, goal_time_parameter}, false);

  EXPECT_DOUBLE_EQ(cmd_timeout, traj_controller_->get_cmd_timeout());
}

/**
 * @brief cmd_timeout must be greater than constraints.goal_time
 */
TEST_P(TrajectoryControllerTestParameterized, decline_false_cmd_timeout)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // zero is default value, just for demonstration
  rclcpp::Parameter cmd_timeout_parameter("cmd_timeout", 1.0);
  rclcpp::Parameter goal_time_parameter("constraints.goal_time", 2.0);
  SetUpAndActivateTrajectoryController(
    executor, {cmd_timeout_parameter, goal_time_parameter}, false);

  EXPECT_DOUBLE_EQ(0.0, traj_controller_->get_cmd_timeout());
}

/**
 * @brief check if no timeout is triggered
 */
// TODO(anyone) make test independent of clock source to use updateControllerAsync
TEST_P(TrajectoryControllerTestParameterized, no_timeout)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // zero is default value, just for demonstration
  rclcpp::Parameter cmd_timeout_parameter("cmd_timeout", 0.0);
  SetUpAndActivateTrajectoryController(executor, {cmd_timeout_parameter}, false);

  size_t n_joints = joint_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 4);

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_joints, state_reference.positions.size());

  // is the trajectory still active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should still hold the points from above
  EXPECT_TRUE(traj_controller_->has_nontrivial_traj());
  EXPECT_NEAR(state_reference.positions[0], points.at(2).at(0), 1e-2);
  EXPECT_NEAR(state_reference.positions[1], points.at(2).at(1), 1e-2);
  EXPECT_NEAR(state_reference.positions[2], points.at(2).at(2), 1e-2);
  // value of velocities is different from above due to spline interpolation
  EXPECT_GT(state_reference.velocities[0], 0.0);
  EXPECT_GT(state_reference.velocities[1], 0.0);
  EXPECT_GT(state_reference.velocities[2], 0.0);

  executor.cancel();
}

/**
 * @brief check if timeout is triggered
 */
// TODO(anyone) make test independent of clock source to use updateControllerAsync
TEST_P(TrajectoryControllerTestParameterized, timeout)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  constexpr double cmd_timeout = 0.1;
  rclcpp::Parameter cmd_timeout_parameter("cmd_timeout", cmd_timeout);
  double kp = 1.0;  // activate feedback control for testing velocity/effort PID
  SetUpAndActivateTrajectoryController(executor, {cmd_timeout_parameter}, false, kp);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*

  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // update until end of trajectory -> no timeout should have occurred
  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 3);
  // is a trajectory active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should have the trajectory with three points
  EXPECT_TRUE(traj_controller_->has_nontrivial_traj());

  // update until timeout should have happened
  updateController(rclcpp::Duration(FIRST_POINT_TIME));

  // after timeout, set_hold_position adds a new trajectory
  // is a trajectory active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should be not more than one point now (from hold position)
  EXPECT_FALSE(traj_controller_->has_nontrivial_traj());
  // should hold last position with zero velocity
  if (traj_controller_->has_position_command_interface())
  {
    expectCommandPoint(points.at(2));
  }
  else
  {
    // no integration to position state interface from velocity/acceleration
    expectCommandPoint(INITIAL_POS_JOINTS);
  }

  executor.cancel();
}

void TrajectoryControllerTest::test_state_publish_rate_target(int target_msg_count)
{
  rclcpp::Parameter state_publish_rate_param(
    "state_publish_rate", static_cast<double>(target_msg_count));
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {state_publish_rate_param});

  auto future_handle = std::async(std::launch::async, [&executor]() -> void { executor.spin(); });

  using control_msgs::msg::JointTrajectoryControllerState;

  const int qos_level = 10;
  int echo_received_counter = 0;
  rclcpp::Subscription<JointTrajectoryControllerState>::SharedPtr subs =
    traj_controller_->get_node()->create_subscription<JointTrajectoryControllerState>(
      controller_name_ + "/state", qos_level,
      [&](JointTrajectoryControllerState::UniquePtr) { ++echo_received_counter; });

  // update for 1second
  auto clock = rclcpp::Clock(RCL_STEADY_TIME);
  const auto start_time = clock.now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(1.0);
  const auto end_time = start_time + wait;
  while (clock.now() < end_time)
  {
    traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  }

  // We may miss the last message since time allowed is exactly the time needed
  EXPECT_NEAR(target_msg_count, echo_received_counter, 2);

  executor.cancel();
}

/**
 * @brief test_state_publish_rate Test that state publish rate matches configure rate
 */
TEST_P(TrajectoryControllerTestParameterized, test_state_publish_rate)
{
  test_state_publish_rate_target(10);
}

TEST_P(TrajectoryControllerTestParameterized, test_lower_state_publish_rate)
{
  test_state_publish_rate_target(1);
}

/**
 * @brief check if use_closed_loop_pid is active
 */
TEST_P(TrajectoryControllerTestParameterized, use_closed_loop_pid)
{
  rclcpp::executors::MultiThreadedExecutor executor;

  SetUpAndActivateTrajectoryController(executor);

  if (
    (traj_controller_->has_velocity_command_interface() &&
     !traj_controller_->has_position_command_interface() &&
     !traj_controller_->has_effort_command_interface() &&
     !traj_controller_->has_acceleration_command_interface() &&
     !traj_controller_->is_open_loop()) ||
    traj_controller_->has_effort_command_interface())
  {
    EXPECT_TRUE(traj_controller_->use_closed_loop_pid_adapter());
  }
}

/**
 * @brief check if velocity error is calculated correctly
 */
TEST_P(TrajectoryControllerTestParameterized, velocity_error)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {}, true);

  size_t n_joints = joint_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points_positions{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}, {{0.3, 0.3, 0.3}}};
  // *INDENT-ON*
  publish(time_from_start, points_positions, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_joints, state_reference.positions.size());
  EXPECT_EQ(n_joints, state_feedback.positions.size());
  EXPECT_EQ(n_joints, state_error.positions.size());
  if (traj_controller_->has_velocity_state_interface())
  {
    EXPECT_EQ(n_joints, state_reference.velocities.size());
    EXPECT_EQ(n_joints, state_feedback.velocities.size());
    EXPECT_EQ(n_joints, state_error.velocities.size());
  }
  if (traj_controller_->has_acceleration_state_interface())
  {
    EXPECT_EQ(n_joints, state_reference.accelerations.size());
    EXPECT_EQ(n_joints, state_feedback.accelerations.size());
    EXPECT_EQ(n_joints, state_error.accelerations.size());
  }

  // no change in state interface should happen
  if (traj_controller_->has_velocity_state_interface())
  {
    EXPECT_EQ(state_feedback.velocities, INITIAL_VEL_JOINTS);
  }
  // is the velocity error correct?
  if (
    traj_controller_->use_closed_loop_pid_adapter()  // always needed for PID controller
    || (traj_controller_->has_velocity_state_interface() &&
        traj_controller_->has_velocity_command_interface()))
  {
    // don't check against a value, because spline interpolation might overshoot depending on
    // interface combinations
    EXPECT_GE(state_error.velocities[0], points_velocities[0][0]);
    EXPECT_GE(state_error.velocities[1], points_velocities[0][1]);
    EXPECT_GE(state_error.velocities[2], points_velocities[0][2]);
  }

  executor.cancel();
}

/**
 * @brief test_jumbled_joint_order Test sending trajectories with a joint order different from
 * internal controller order
 */
TEST_P(TrajectoryControllerTestParameterized, test_jumbled_joint_order)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);
  std::vector<double> points_positions = {1.0, 2.0, 3.0};
  std::vector<size_t> jumble_map = {1, 2, 0};
  double dt = 0.25;
  {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    const std::vector<std::string> jumbled_joint_names{
      joint_names_[jumble_map[0]], joint_names_[jumble_map[1]], joint_names_[jumble_map[2]]};

    traj_msg.joint_names = jumbled_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(dt);
    traj_msg.points[0].positions.resize(3);
    traj_msg.points[0].positions[0] = points_positions.at(jumble_map[0]);
    traj_msg.points[0].positions[1] = points_positions.at(jumble_map[1]);
    traj_msg.points[0].positions[2] = points_positions.at(jumble_map[2]);
    traj_msg.points[0].velocities.resize(3);
    traj_msg.points[0].accelerations.resize(3);

    for (size_t dof = 0; dof < 3; dof++)
    {
      traj_msg.points[0].velocities[dof] =
        (traj_msg.points[0].positions[dof] - joint_pos_[jumble_map[dof]]) / dt;
      traj_msg.points[0].accelerations[dof] =
        (traj_msg.points[0].velocities[dof] - joint_vel_[jumble_map[dof]]) / dt;
    }

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(dt));

  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(points_positions.at(0), joint_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points_positions.at(1), joint_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points_positions.at(2), joint_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // if use_closed_loop_pid_adapter_==false: we expect desired velocities from direct sampling
    // if use_closed_loop_pid_adapter_==true: we expect desired velocities, because we use PID with
    // feedforward term only
    EXPECT_GT(0.0, joint_vel_[0]);
    EXPECT_GT(0.0, joint_vel_[1]);
    EXPECT_GT(0.0, joint_vel_[2]);
  }

  if (traj_controller_->has_acceleration_command_interface())
  {
    EXPECT_GT(0.0, joint_acc_[0]);
    EXPECT_GT(0.0, joint_acc_[1]);
    EXPECT_GT(0.0, joint_acc_[2]);
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // effort should be nonzero, because we use PID with feedforward term
    EXPECT_GT(0.0, joint_eff_[0]);
    EXPECT_GT(0.0, joint_eff_[1]);
    EXPECT_GT(0.0, joint_eff_[2]);
  }
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled
 * joints
 */
TEST_P(TrajectoryControllerTestParameterized, test_partial_joint_list)
{
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {partial_joints_parameters});

  const double initial_joint1_cmd = joint_pos_[0];
  const double initial_joint2_cmd = joint_pos_[1];
  const double initial_joint3_cmd = joint_pos_[2];
  const double dt = 0.25;
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<size_t> jumble_map = {1, 0};
    std::vector<std::string> partial_joint_names{
      joint_names_[jumble_map[0]], joint_names_[jumble_map[1]]};
    traj_msg.joint_names = partial_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(dt);
    traj_msg.points[0].positions.resize(2);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 1.0;
    traj_msg.points[0].velocities.resize(2);
    traj_msg.points[0].accelerations.resize(2);
    for (size_t dof = 0; dof < 2; dof++)
    {
      traj_msg.points[0].velocities[dof] =
        (traj_msg.points[0].positions[dof] - joint_pos_[jumble_map[dof]]) / dt;
      traj_msg.points[0].accelerations[dof] =
        (traj_msg.points[0].velocities[dof] - joint_vel_[jumble_map[dof]]) / dt;
    }

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(dt));

  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(traj_msg.points[0].positions[1], joint_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(traj_msg.points[0].positions[0], joint_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(initial_joint3_cmd, joint_pos_[2], COMMON_THRESHOLD)
      << "Joint 3 command should be current position";
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // estimate the sign of the velocity
    // joint rotates forward
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[0] - initial_joint2_cmd, joint_vel_[0]));
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[1] - initial_joint1_cmd, joint_vel_[1]));
    EXPECT_NEAR(0.0, joint_vel_[2], COMMON_THRESHOLD)
      << "Joint 3 velocity should be 0.0 since it's not in the goal";
  }

  if (traj_controller_->has_acceleration_command_interface())
  {
    // estimate the sign of the acceleration
    // joint rotates forward
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[0] - initial_joint2_cmd, joint_acc_[0]))
      << "Joint1: " << traj_msg.points[0].positions[0] - initial_joint2_cmd << " vs. "
      << joint_acc_[0];
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[1] - initial_joint1_cmd, joint_acc_[1]))
      << "Joint2: " << traj_msg.points[0].positions[1] - initial_joint1_cmd << " vs. "
      << joint_acc_[1];
    EXPECT_NEAR(0.0, joint_acc_[2], COMMON_THRESHOLD)
      << "Joint 3 acc should be 0.0 since it's not in the goal";
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // estimate the sign of the effort
    // joint rotates forward
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[0] - initial_joint2_cmd, joint_eff_[0]));
    EXPECT_TRUE(
      is_same_sign_or_zero(traj_msg.points[0].positions[1] - initial_joint1_cmd, joint_eff_[1]));
    EXPECT_NEAR(0.0, joint_eff_[2], COMMON_THRESHOLD)
      << "Joint 3 effort should be 0.0 since it's not in the goal";
  }

  executor.cancel();
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled
 * joints without allow_partial_joints_goal
 */
TEST_P(TrajectoryControllerTestParameterized, test_partial_joint_list_not_allowed)
{
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {partial_joints_parameters});

  const double initial_joint1_cmd = joint_pos_[0];
  const double initial_joint2_cmd = joint_pos_[1];
  const double initial_joint3_cmd = joint_pos_[2];
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names{joint_names_[1], joint_names_[0]};
    traj_msg.joint_names = partial_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(2);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 1.0;
    traj_msg.points[0].velocities.resize(2);
    traj_msg.points[0].velocities[0] = 2.0;
    traj_msg.points[0].velocities[1] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.5 seconds
  updateControllerAsync(rclcpp::Duration::from_seconds(0.25));

  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(initial_joint1_cmd, joint_pos_[0], COMMON_THRESHOLD)
      << "All joints command should be current position because goal was rejected";
    EXPECT_NEAR(initial_joint2_cmd, joint_pos_[1], COMMON_THRESHOLD)
      << "All joints command should be current position because goal was rejected";
    EXPECT_NEAR(initial_joint3_cmd, joint_pos_[2], COMMON_THRESHOLD)
      << "All joints command should be current position because goal was rejected";
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    EXPECT_NEAR(INITIAL_VEL_JOINTS[0], joint_vel_[0], COMMON_THRESHOLD)
      << "All joints velocities should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_VEL_JOINTS[1], joint_vel_[1], COMMON_THRESHOLD)
      << "All joints velocities should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_VEL_JOINTS[2], joint_vel_[2], COMMON_THRESHOLD)
      << "All joints velocities should be 0.0 because goal was rejected";
  }

  if (traj_controller_->has_acceleration_command_interface())
  {
    EXPECT_NEAR(INITIAL_ACC_JOINTS[0], joint_acc_[0], COMMON_THRESHOLD)
      << "All joints accelerations should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_ACC_JOINTS[1], joint_acc_[1], COMMON_THRESHOLD)
      << "All joints accelerations should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_ACC_JOINTS[2], joint_acc_[2], COMMON_THRESHOLD)
      << "All joints accelerations should be 0.0 because goal was rejected";
  }

  if (traj_controller_->has_effort_command_interface())
  {
    EXPECT_NEAR(INITIAL_EFF_JOINTS[0], joint_eff_[0], COMMON_THRESHOLD)
      << "All joints efforts should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_EFF_JOINTS[1], joint_eff_[1], COMMON_THRESHOLD)
      << "All joints efforts should be 0.0 because goal was rejected";
    EXPECT_NEAR(INITIAL_EFF_JOINTS[2], joint_eff_[2], COMMON_THRESHOLD)
      << "All joints efforts should be 0.0 because goal was rejected";
  }

  executor.cancel();
}

/**
 * @brief invalid_message Test mismatched joint and reference vector sizes
 */
TEST_P(TrajectoryControllerTestParameterized, invalid_message)
{
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);
  rclcpp::Parameter allow_integration_parameters("allow_integration_in_goal_trajectories", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(
    executor, {partial_joints_parameters, allow_integration_parameters});

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // Incompatible joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name"};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // empty message
  traj_msg = good_traj_msg;
  traj_msg.points.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Effort is not supported in trajectory message
  traj_msg = good_traj_msg;
  traj_msg.points[0].effort = {1.0, 2.0, 3.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Non-strictly increasing waypoint times
  traj_msg = good_traj_msg;
  traj_msg.points.push_back(traj_msg.points.front());
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // End time in the past
  traj_msg = good_traj_msg;
  traj_msg.header.stamp = rclcpp::Time(1);
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // End time in the future
  traj_msg = good_traj_msg;
  traj_msg.header.stamp = traj_controller_->get_node()->now();
  traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(10);
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief Test invalid velocity at trajectory end with parameter set to false
 */
TEST_P(
  TrajectoryControllerTestParameterized,
  expect_invalid_when_message_with_nonzero_end_velocity_and_when_param_false)
{
  rclcpp::Parameter nonzero_vel_parameters("allow_nonzero_velocity_at_trajectory_end", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {nonzero_vel_parameters});

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0);

  // empty message (no throw!)
  ASSERT_NO_THROW(traj_controller_->validate_trajectory_msg(traj_msg));
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Nonzero velocity at trajectory end!
  traj_msg.points.resize(1);
  traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  traj_msg.points[0].positions.resize(1);
  traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  traj_msg.points[0].velocities.resize(1);
  traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief missing_positions_message_accepted Test mismatched joint and reference vector sizes
 *
 * @note With allow_integration_in_goal_trajectories parameter trajectory missing position or
 * velocities are accepted
 */
TEST_P(TrajectoryControllerTestParameterized, missing_positions_message_accepted)
{
  rclcpp::Parameter allow_integration_parameters("allow_integration_in_goal_trajectories", true);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {allow_integration_parameters});

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  good_traj_msg.points[0].accelerations.resize(1);
  good_traj_msg.points[0].accelerations = {1.0, 2.0, 3.0};
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position and velocity data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].velocities.clear();
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

  // All empty
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].velocities.clear();
  traj_msg.points[0].accelerations.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief test_trajectory_replace Test replacing an existing trajectory
 */
TEST_P(TrajectoryControllerTestParameterized, test_trajectory_replace)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  SetUpAndActivateTrajectoryController(executor, {partial_joints_parameters});

  std::vector<std::vector<double>> points_old{{{2., 3., 4.}}};
  std::vector<std::vector<double>> points_old_velocities{{{0.2, 0.3, 0.4}}};
  std::vector<std::vector<double>> points_partial_new{{1.5}};
  std::vector<std::vector<double>> points_partial_new_velocities{{0.15}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old, rclcpp::Time(), {}, points_old_velocities);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired.positions = {points_old[0].begin(), points_old[0].end()};
  expected_actual.velocities = {points_old_velocities[0].begin(), points_old_velocities[0].end()};
  expected_desired.velocities = {points_old_velocities[0].begin(), points_old_velocities[0].end()};
  //  Check that we reached end of points_old trajectory
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory");
  points_partial_new_velocities[0][0] =
    std::copysign(0.15, points_partial_new[0][0] - joint_state_pos_[0]);
  publish(time_from_start, points_partial_new, rclcpp::Time(), {}, points_partial_new_velocities);

  // Replaced trajectory is a mix of previous and current goal
  expected_desired.positions[0] = points_partial_new[0][0];
  expected_desired.positions[1] = points_old[0][1];
  expected_desired.positions[2] = points_old[0][2];
  expected_desired.velocities[0] = points_partial_new_velocities[0][0];
  expected_desired.velocities[1] = 0.0;
  expected_desired.velocities[2] = 0.0;
  expected_actual = expected_desired;
  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1, end_time);
}

/**
 * @brief test_ignore_old_trajectory Sending an old trajectory replacing an existing trajectory
 */
TEST_P(TrajectoryControllerTestParameterized, test_ignore_old_trajectory)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  // TODO(anyone): add expectations for velocities and accelerations
  std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new{{{-1., -2., -3.}}};

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the future");
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old, rclcpp::Time());
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0] trajectory
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the past");
  //  New trajectory will end before current time
  rclcpp::Time new_traj_start =
    rclcpp::Clock(RCL_STEADY_TIME).now() - delay - std::chrono::milliseconds(100);
  expected_actual.positions = {points_old[1].begin(), points_old[1].end()};
  expected_desired = expected_actual;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1, end_time);
}

TEST_P(TrajectoryControllerTestParameterized, test_ignore_partial_old_trajectory)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new{{{-1., -2., -3.}, {-2., -4., -6.}}};
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};

  // send points_old and wait to reach first point
  publish(time_from_start, points_old, rclcpp::Time());
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  // send points_new before the old trajectory is finished
  RCLCPP_INFO(
    traj_controller_->get_node()->get_logger(), "Sending new trajectory partially in the past");
  //  New trajectory first point is in the past, second is in the future
  rclcpp::Time new_traj_start = end_time - delay - std::chrono::milliseconds(100);
  publish(time_from_start, points_new, new_traj_start);
  // it should not have accepted the new goal but finish the old one
  expected_actual.positions = {points_old[1].begin(), points_old[1].end()};
  expected_desired.positions = {points_old[1].begin(), points_old[1].end()};
  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1, end_time);
}

TEST_P(TrajectoryControllerTestParameterized, test_execute_partial_traj_in_future)
{
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {partial_joints_parameters});

  RCLCPP_WARN(
    traj_controller_->get_node()->get_logger(),
    "Test disabled until current_trajectory is taken into account when adding a new trajectory.");
  // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/
  // joint_trajectory_controller/include/joint_trajectory_controller/init_joint_trajectory.h#L149
  return;

  // *INDENT-OFF*
  std::vector<std::vector<double>> full_traj{{{2., 3., 4.}, {4., 6., 8.}}};
  std::vector<std::vector<double>> full_traj_velocities{{{0.2, 0.3, 0.4}, {0.4, 0.6, 0.8}}};
  std::vector<std::vector<double>> partial_traj{{{-1., -2.}, {-2., -4}}};
  std::vector<std::vector<double>> partial_traj_velocities{{{-0.1, -0.2}, {-0.2, -0.4}}};
  // *INDENT-ON*
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration points_delay{rclcpp::Duration(delay)};
  // Send full trajectory
  publish(points_delay, full_traj, rclcpp::Time(), {}, full_traj_velocities);
  // Sleep until first waypoint of full trajectory

  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {full_traj[0].begin(), full_traj[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory and are starting points_old[1]
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  // Send partial trajectory starting after full trajecotry is complete
  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the future");
  publish(
    points_delay, partial_traj, rclcpp::Clock(RCL_STEADY_TIME).now() + delay * 2, {},
    partial_traj_velocities);
  // Wait until the end start and end of partial traj

  expected_actual.positions = {partial_traj.back()[0], partial_traj.back()[1], full_traj.back()[2]};
  expected_desired = expected_actual;

  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(delay * (2 + 2)), 0.1, end_time);
}

TEST_P(TrajectoryControllerTestParameterized, test_jump_when_state_tracking_error_updated)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  // default if false so it will not be actually set parameter
  rclcpp::Parameter is_open_loop_parameters("open_loop_control", false);
  SetUpAndActivateTrajectoryController(executor, {is_open_loop_parameters}, true);

  if (traj_controller_->has_position_command_interface() == false)
  {
    // only makes sense with position command interface
    return;
  }

  // goal setup
  std::vector<double> first_goal = {3.3, 4.4, 5.5};
  std::vector<std::vector<double>> first_goal_velocities = {{0.33, 0.44, 0.55}};
  std::vector<double> second_goal = {6.6, 8.8, 11.0};
  std::vector<std::vector<double>> second_goal_velocities = {{0.66, 0.88, 1.1}};
  double state_from_command_offset = 0.3;

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  double trajectory_frac = rclcpp::Duration::from_seconds(0.01).seconds() /
                           (time_from_start.sec + time_from_start.nanosec * 1e-9);
  std::vector<std::vector<double>> points{{first_goal}};
  publish(
    time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME), {}, first_goal_velocities);
  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1));

  // JTC is executing trajectory in open-loop therefore:
  // - internal state does not have to be updated (in this test-case it shouldn't)
  // - internal command is updated
  EXPECT_NEAR(INITIAL_POS_JOINT1, joint_state_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  joint_state_pos_[0] = first_goal[0] - state_from_command_offset;

  // Move joint further in the same direction as before (to the second goal)
  points = {{second_goal}};
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, second_goal_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there should be a "jump" in opposite direction from command
  // (towards the state value)
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
  auto end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // Expect backward commands at first, consider advancement of the trajectory
  // exact value is not directly predictable, because of the spline interpolation -> increase
  // tolerance
  EXPECT_NEAR(
    joint_state_pos_[0] + (second_goal[0] - joint_state_pos_[0]) * trajectory_frac, joint_pos_[0],
    0.1);
  EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_LT(joint_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_LT(joint_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_LT(joint_pos_[0], first_goal[0]);

  // Finally the second goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  joint_state_pos_[0] = second_goal[0] - state_from_command_offset;

  // Move joint back to the first goal
  points = {{first_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there should be a "jump" in the goal direction from command
  // (towards the state value)
  EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // Expect backward commands at first, consider advancement of the trajectory
  EXPECT_NEAR(
    joint_state_pos_[0] + (first_goal[0] - joint_state_pos_[0]) * trajectory_frac, joint_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
  EXPECT_GT(joint_pos_[0], first_goal[0]);

  // Finally the first goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, test_no_jump_when_state_tracking_error_not_updated)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  // set open loop to true, this should change behavior from above
  rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, {is_open_loop_parameters}, true);

  if (traj_controller_->has_position_command_interface() == false)
  {
    // only makes sense with position command interface
    return;
  }

  // goal setup
  std::vector<double> first_goal = {3.3, 4.4, 5.5};
  std::vector<double> second_goal = {6.6, 8.8, 11.0};
  double state_from_command_offset = 0.3;

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  double trajectory_frac = rclcpp::Duration::from_seconds(0.01).seconds() /
                           (time_from_start.sec + time_from_start.nanosec * 1e-9);
  std::vector<std::vector<double>> points{{first_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1));

  // JTC is executing trajectory in open-loop therefore:
  // - internal state does not have to be updated (in this test-case it shouldn't)
  // - internal command is updated
  EXPECT_NEAR(INITIAL_POS_JOINT1, joint_state_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  joint_state_pos_[0] = first_goal[0] - state_from_command_offset;

  // Move joint further in the same direction as before (to the second goal)
  points = {{second_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there **should not** be a "jump" in opposite direction from
  // command (towards the state value)
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
  auto end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // There should not be backward commands
  EXPECT_NEAR(
    first_goal[0] + (second_goal[0] - first_goal[0]) * trajectory_frac, joint_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  EXPECT_LT(joint_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  EXPECT_LT(joint_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  EXPECT_LT(joint_pos_[0], second_goal[0]);

  // Finally the second goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  joint_state_pos_[0] = second_goal[0] - state_from_command_offset;

  // Move joint back to the first goal
  points = {{first_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there **should not** be a "jump" in the goal direction from
  // command (towards the state value)
  EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  // There should not be a jump toward commands
  EXPECT_NEAR(
    second_goal[0] + (first_goal[0] - second_goal[0]) * trajectory_frac, joint_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_LT(joint_pos_[0], second_goal[0]);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  EXPECT_LT(joint_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(joint_pos_[0], first_goal[0]);
  EXPECT_LT(joint_pos_[0], second_goal[0]);

  // Finally the first goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

  executor.cancel();
}

// Testing that values are read from state interfaces when hardware is started for the first
// time and hardware state has offset --> this is indicated by NaN values in command interfaces
TEST_P(TrajectoryControllerTestParameterized, test_hw_states_has_offset_first_controller_start)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);

  // set command values to NaN
  std::vector<double> initial_pos_cmd{3, std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> initial_vel_cmd{3, std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> initial_acc_cmd{3, std::numeric_limits<double>::quiet_NaN()};

  SetUpAndActivateTrajectoryController(
    executor, {is_open_loop_parameters}, true, 0., 1., false, initial_pos_cmd, initial_vel_cmd,
    initial_acc_cmd);

  // no call of update method, so the values should be read from state interfaces
  // (command interface are NaN)

  auto current_state_when_offset = traj_controller_->get_current_state_when_offset();

  for (size_t i = 0; i < 3; ++i)
  {
    EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);

    // check velocity
    if (traj_controller_->has_velocity_state_interface())
    {
      EXPECT_EQ(current_state_when_offset.velocities[i], joint_state_vel_[i]);
    }

    // check acceleration
    if (traj_controller_->has_acceleration_state_interface())
    {
      EXPECT_EQ(current_state_when_offset.accelerations[i], joint_state_acc_[i]);
    }
  }

  executor.cancel();
}

// Testing that values are read from command interfaces when hardware is started after some values
// are set on the hardware commands
TEST_P(TrajectoryControllerTestParameterized, test_hw_states_has_offset_later_controller_start)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);

  // set command values to arbitrary values
  std::vector<double> initial_pos_cmd, initial_vel_cmd, initial_acc_cmd;
  for (size_t i = 0; i < 3; ++i)
  {
    initial_pos_cmd.push_back(3.1 + static_cast<double>(i));
    initial_vel_cmd.push_back(0.25 + static_cast<double>(i));
    initial_acc_cmd.push_back(0.02 + static_cast<double>(i) / 10.0);
  }
  SetUpAndActivateTrajectoryController(
    executor, {is_open_loop_parameters}, true, 0., 1., false, initial_pos_cmd, initial_vel_cmd,
    initial_acc_cmd);

  // no call of update method, so the values should be read from command interfaces

  auto current_state_when_offset = traj_controller_->get_current_state_when_offset();

  for (size_t i = 0; i < 3; ++i)
  {
    // check position
    if (traj_controller_->has_position_command_interface())
    {
      // check velocity
      if (traj_controller_->has_velocity_state_interface())
      {
        if (traj_controller_->has_velocity_command_interface())
        {
          // check acceleration
          if (traj_controller_->has_acceleration_state_interface())
          {
            if (traj_controller_->has_acceleration_command_interface())
            {
              // should have set it to last position + velocity + acceleration command
              EXPECT_EQ(current_state_when_offset.positions[i], initial_pos_cmd[i]);
              EXPECT_EQ(current_state_when_offset.velocities[i], initial_vel_cmd[i]);
              EXPECT_EQ(current_state_when_offset.accelerations[i], initial_acc_cmd[i]);
            }
            else
            {
              // should have set it to the state interface instead
              EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);
              EXPECT_EQ(current_state_when_offset.velocities[i], joint_state_vel_[i]);
              EXPECT_EQ(current_state_when_offset.accelerations[i], joint_state_acc_[i]);
            }
          }
          else
          {
            // should have set it to last position + velocity command
            EXPECT_EQ(current_state_when_offset.positions[i], initial_pos_cmd[i]);
            EXPECT_EQ(current_state_when_offset.velocities[i], initial_vel_cmd[i]);
          }
        }
        else
        {
          // should have set it to the state interface instead
          EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);
          EXPECT_EQ(current_state_when_offset.velocities[i], joint_state_vel_[i]);
        }
      }
      else
      {
        // should have set it to last position command
        EXPECT_EQ(current_state_when_offset.positions[i], initial_pos_cmd[i]);
      }
    }
    else
    {
      // should have set it to the state interface instead
      EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);
    }
  }

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, test_state_tolerances_fail)
{
  // set joint tolerance parameters
  const double state_tol = 0.0001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint2.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint3.trajectory", state_tol)};

  rclcpp::executors::MultiThreadedExecutor executor;
  double kp = 1.0;  // activate feedback control for testing velocity/effort PID
  SetUpAndActivateTrajectoryController(executor, params, true, kp);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(100);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // it should have aborted and be holding now
  expectCommandPoint(joint_state_pos_);
}

TEST_P(TrajectoryControllerTestParameterized, test_goal_tolerances_fail)
{
  // set joint tolerance parameters
  const double goal_tol = 0.1;
  // set very small goal_time so that goal_time is violated
  const double goal_time = 0.000001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.goal", goal_tol),
    rclcpp::Parameter("constraints.joint2.goal", goal_tol),
    rclcpp::Parameter("constraints.joint3.goal", goal_tol),
    rclcpp::Parameter("constraints.goal_time", goal_time)};

  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, params, true, 1.0);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(100);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);
  auto end_time = updateControllerAsync(rclcpp::Duration(4 * FIRST_POINT_TIME));

  // it should have aborted and be holding now
  expectCommandPoint(joint_state_pos_);

  // what happens if we wait longer but it harms the tolerance again?
  auto hold_position = joint_state_pos_;
  joint_state_pos_.at(0) = -3.3;
  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME), end_time);
  // it should be still holding the old point
  expectCommandPoint(hold_position);
}

// position controllers
INSTANTIATE_TEST_SUITE_P(
  PositionTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(std::vector<std::string>({"position"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// position_velocity controllers
INSTANTIATE_TEST_SUITE_P(
  PositionVelocityTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// position_velocity_acceleration controllers
INSTANTIATE_TEST_SUITE_P(
  PositionVelocityAccelerationTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// only velocity controller
INSTANTIATE_TEST_SUITE_P(
  OnlyVelocityTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"velocity"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"velocity"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// only effort controller
INSTANTIATE_TEST_SUITE_P(
  OnlyEffortTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"effort"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"effort"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

/**
 * @brief see if parameter validation is correct
 *
 * Note: generate_parameter_library validates parameters itself during on_init() method, but
 * combinations of parameters are checked from JTC during on_configure()
 */
TEST_F(TrajectoryControllerTest, incorrect_initialization_using_interface_parameters)
{
  // command interfaces: empty
  command_interface_types_ = {};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::OK);
  auto state = traj_controller_->get_node()->configure();
  EXPECT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);

  // command interfaces: bad_name
  command_interface_types_ = {"bad_name"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // command interfaces: effort has to be only
  command_interface_types_ = {"effort", "position"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // command interfaces: velocity - position not present
  command_interface_types_ = {"velocity", "acceleration"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // command interfaces: acceleration without position and velocity
  command_interface_types_ = {"acceleration"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // state interfaces: empty
  state_interface_types_ = {};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // state interfaces: cannot not be effort
  state_interface_types_ = {"effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // state interfaces: bad name
  state_interface_types_ = {"bad_name"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // state interfaces: velocity - position not present
  state_interface_types_ = {"velocity"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"velocity", "acceleration"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // state interfaces: acceleration without position and velocity
  state_interface_types_ = {"acceleration"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // velocity-only command interface: position - velocity not present
  command_interface_types_ = {"velocity"};
  state_interface_types_ = {"position"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::OK);
  state = traj_controller_->get_node()->configure();
  EXPECT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  state_interface_types_ = {"velocity"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // effort-only command interface: position - velocity not present
  command_interface_types_ = {"effort"};
  state_interface_types_ = {"position"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::OK);
  state = traj_controller_->get_node()->configure();
  EXPECT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  state_interface_types_ = {"velocity"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
}
