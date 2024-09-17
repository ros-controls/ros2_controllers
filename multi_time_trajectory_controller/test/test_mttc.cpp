// Copyright 2024 ros2_control Development Team
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

#include <gtest/gtest.h>
#include <chrono>
#include <cstddef>
#include <limits>
#include <vector>

#include <rclcpp/parameter.hpp>
#include "control_msgs/msg/axis_trajectory_point.hpp"
#include "control_msgs/msg/multi_axis_trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "multi_time_trajectory_controller/multi_time_trajectory_controller.hpp"
#include "test_mttc_utils.hpp"

#include "test_assets.hpp"

using lifecycle_msgs::msg::State;
using test_mttc::TrajectoryControllerTest;
using test_mttc::TrajectoryControllerTestParameterized;

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
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.5 * 1e9)), rclcpp::Duration::from_seconds(0.5));

  // hw position == 0 because controller is not activated
  EXPECT_EQ(0.0, axis_pos_[0]);
  EXPECT_EQ(0.0, axis_pos_[1]);
  EXPECT_EQ(0.0, axis_pos_[2]);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  compare_axes(axis_names_, axis_names_);
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names_with_command_axes)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // set command_axes parameter different than axis_names_
  const rclcpp::Parameter command_axis_names_param("command_axes", command_axis_names_);
  SetUpTrajectoryController(executor, {command_axis_names_param});

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  compare_axes(axis_names_, command_axis_names_);
}

TEST_P(TrajectoryControllerTestParameterized, activate)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  auto cmd_if_conf = traj_controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), axis_names_.size() * command_interface_types_.size());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = traj_controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), axis_names_.size() * state_interface_types_.size());
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
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocity);
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
  EXPECT_EQ(INITIAL_POS_AXIS1, axis_pos_[0]);
  EXPECT_EQ(INITIAL_POS_AXIS2, axis_pos_[1]);
  EXPECT_EQ(INITIAL_POS_AXIS3, axis_pos_[2]);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocity);
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
    EXPECT_NEAR(points.at(0).at(0), axis_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points.at(0).at(1), axis_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points.at(0).at(2), axis_pos_[2], COMMON_THRESHOLD);
  }

  // deactivate
  std::vector<double> deactivated_positions{axis_pos_[0], axis_pos_[1], axis_pos_[2]};
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

  ASSERT_FALSE(state == nullptr);

  std::size_t n_axes = axis_names_.size();

  for (unsigned int i = 0; i < n_axes; ++i)
  {
    EXPECT_EQ(axis_names_[i], state->axis_names[i]);
  }

  for (std::size_t i = 0; i < 3; ++i)
  {
    // No trajectory by default, no reference state or error
    EXPECT_TRUE(
      std::isnan(state->references[i].position) ||
      state->references[i].position == INITIAL_POS_AXES[i]);
    EXPECT_TRUE(
      std::isnan(state->references[i].velocity) ||
      state->references[i].velocity == INITIAL_VEL_AXES[i]);
    EXPECT_TRUE(
      std::isnan(state->references[i].acceleration) ||
      state->references[i].acceleration == INITIAL_EFF_AXES[i]);
  }

  for (std::size_t i = 0; i < DOF; ++i)
  {
    EXPECT_EQ(state->errors[i].position, 0);
    EXPECT_TRUE(std::isnan(state->errors[i].velocity) || state->errors[i].velocity == 0);
    EXPECT_TRUE(std::isnan(state->errors[i].acceleration) || state->errors[i].acceleration == 0);

    // expect feedback including all state_interfaces
    EXPECT_EQ(n_axes, state->feedbacks.size());
    if (
      std::find(state_interface_types_.begin(), state_interface_types_.end(), "velocity") ==
      state_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->feedbacks[i].velocity));
    }
    if (
      std::find(state_interface_types_.begin(), state_interface_types_.end(), "acceleration") ==
      state_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->feedbacks[i].acceleration));
    }

    EXPECT_EQ(n_axes, state->outputs.size());

    // expect output including all command_interfaces
    if (
      std::find(command_interface_types_.begin(), command_interface_types_.end(), "position") ==
      command_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->outputs[i].position));
    }

    if (
      std::find(command_interface_types_.begin(), command_interface_types_.end(), "velocity") ==
      command_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->outputs[i].velocity));
    }

    if (
      std::find(command_interface_types_.begin(), command_interface_types_.end(), "acceleration") ==
      command_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->outputs[i].acceleration));
    }

    if (
      std::find(command_interface_types_.begin(), command_interface_types_.end(), "effort") ==
      command_interface_types_.end())
    {
      EXPECT_TRUE(std::isnan(state->outputs[i].effort));
    }
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
    auto gain_0 = pids.at(0)->getGains();
    EXPECT_EQ(gain_0.p_gain_, 0.0);

    double kp = 1.0;
    SetPidParameters(kp);
    updateControllerAsync();

    pids = traj_controller_->get_pids();
    EXPECT_EQ(pids.size(), 3);
    gain_0 = pids.at(0)->getGains();
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
    for (std::size_t i = 0; i < axis_names_.size(); ++i)
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
    rclcpp::Parameter("constraints.axis1.trajectory", 1.0),
    rclcpp::Parameter("constraints.axis2.trajectory", 2.0),
    rclcpp::Parameter("constraints.axis3.trajectory", 3.0),
    rclcpp::Parameter("constraints.axis1.goal", 10.0),
    rclcpp::Parameter("constraints.axis2.goal", 20.0),
    rclcpp::Parameter("constraints.axis3.goal", 30.0)};
  for (const auto & param : new_tolerances)
  {
    traj_controller_->get_node()->set_parameter(param);
  }
  updateControllerAsync();
  {
    auto tols = traj_controller_->get_tolerances();
    EXPECT_EQ(tols.goal_time_tolerance, 1.0);
    for (std::size_t i = 0; i < axis_names_.size(); ++i)
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
  std::vector<double> initial_position{INITIAL_POS_AXIS1, INITIAL_POS_AXIS2, INITIAL_POS_AXIS3};

  expectCommandPoint(initial_position);

  executor.cancel();
}

// Floating-point value comparison threshold
const double EPS = 1e-6;

/**
 * @brief check if calculated trajectory error is correct (angle wraparound) for continuous axes
 */
TEST_P(TrajectoryControllerTestParameterized, compute_error_angle_wraparound_true)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(
    executor, params, true, 0.0, 1.0, INITIAL_POS_AXES, INITIAL_VEL_AXES, INITIAL_ACC_AXES,
    INITIAL_EFF_AXES, test_mttc::urdf_rrrbot_continuous);

  std::size_t n_axes = axis_names_.size();

  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  std::vector<std::vector<double>> points_acceleration{
    {{0.1, 0.1, 0.1}}, {{0.5, 0.5, 0.5}}, {{0.6, 0.6, 0.6}}};
  // *INDENT-ON*

  std::size_t n_points = points.size();

  for (std::size_t point_num = 0; point_num < n_points; ++point_num)
  {
    std::vector<control_msgs::msg::AxisTrajectoryPoint> error, current, desired;
    for (std::size_t axis = 0; axis < n_axes; ++axis)
    {
      control_msgs::msg::AxisTrajectoryPoint point;
      point.position = points[point_num][axis];
      point.position = points_velocity[point_num][axis];
      point.position = points_acceleration[point_num][axis];
      current.push_back(point);
    }

    // zero error
    desired = current;
    traj_controller_->testable_compute_error(error, current, desired);
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      EXPECT_NEAR(error[i].position, 0., EPS);
      if (
        traj_controller_->has_velocity_state_interface() &&
        (traj_controller_->has_velocity_command_interface() ||
         traj_controller_->has_effort_command_interface()))
      {
        // expect: error.velocity = desired.velocity - current.velocity;
        EXPECT_NEAR(error[i].velocity, 0., EPS);
      }
      if (
        traj_controller_->has_acceleration_state_interface() &&
        traj_controller_->has_acceleration_command_interface())
      {
        // expect: error.acceleration = desired.acceleration - current.acceleration;
        EXPECT_NEAR(error[i].acceleration, 0., EPS);
      }
    }

    // angle wraparound of position error
    desired[0].position += 3.0 * M_PI_2;
    desired[0].velocity += 1.0;
    desired[0].acceleration += 1.0;
    traj_controller_->testable_compute_error(error, current, desired);
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      if (i == 0)
      {
        EXPECT_NEAR(error[i].position, desired[i].position - current[i].position - 2.0 * M_PI, EPS);
      }
      else
      {
        EXPECT_NEAR(error[i].position, desired[i].position - current[i].position, EPS);
      }

      if (
        traj_controller_->has_velocity_state_interface() &&
        (traj_controller_->has_velocity_command_interface() ||
         traj_controller_->has_effort_command_interface()))
      {
        // expect: error.velocity = desired.velocity - current.velocity;
        EXPECT_NEAR(error[i].velocity, desired[i].velocity - current[i].velocity, EPS);
      }
      if (
        traj_controller_->has_acceleration_state_interface() &&
        traj_controller_->has_acceleration_command_interface())
      {
        // expect: error.acceleration = desired.acceleration - current.acceleration;
        EXPECT_NEAR(error[i].acceleration, desired[i].acceleration - current[i].acceleration, EPS);
      }
    }
  }
  executor.cancel();
}

/**
 * @brief check if calculated trajectory error is correct (no angle wraparound) for revolute axes
 */
TEST_P(TrajectoryControllerTestParameterized, compute_error_angle_wraparound_false)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(
    executor, params, true, 0.0, 1.0, INITIAL_POS_AXES, INITIAL_VEL_AXES, INITIAL_ACC_AXES,
    INITIAL_EFF_AXES, test_mttc::urdf_rrrbot_revolute);

  std::size_t n_axes = axis_names_.size();

  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  std::vector<std::vector<double>> points_acceleration{
    {{0.1, 0.1, 0.1}}, {{0.5, 0.5, 0.5}}, {{0.6, 0.6, 0.6}}};
  // *INDENT-ON*

  std::size_t n_points = points.size();

  for (std::size_t point_num = 0; point_num < n_points; ++point_num)
  {
    std::vector<control_msgs::msg::AxisTrajectoryPoint> error, current, desired;
    for (std::size_t axis = 0; axis < n_axes; ++axis)
    {
      control_msgs::msg::AxisTrajectoryPoint point;
      point.position = points[point_num][axis];
      point.position = points_velocity[point_num][axis];
      point.position = points_acceleration[point_num][axis];
      current.push_back(point);
    }

    // zero error
    desired = current;
    traj_controller_->testable_compute_error(error, current, desired);
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      EXPECT_NEAR(error[i].position, 0., EPS);
      if (
        traj_controller_->has_velocity_state_interface() &&
        (traj_controller_->has_velocity_command_interface() ||
         traj_controller_->has_effort_command_interface()))
      {
        // expect: error.velocity = desired.velocity - current.velocity;
        EXPECT_NEAR(error[i].velocity, 0., EPS);
      }
      if (
        traj_controller_->has_acceleration_state_interface() &&
        traj_controller_->has_acceleration_command_interface())
      {
        // expect: error.acceleration = desired.acceleration - current.acceleration;
        EXPECT_NEAR(error[i].acceleration, 0., EPS);
      }
    }

    // no normalization of position error
    desired[0].position += 3.0 * M_PI_4;
    desired[0].velocity += 1.0;
    desired[0].acceleration += 1.0;

    traj_controller_->testable_compute_error(error, current, desired);
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      EXPECT_NEAR(error[i].position, desired[i].position - current[i].position, EPS);
      if (
        traj_controller_->has_velocity_state_interface() &&
        (traj_controller_->has_velocity_command_interface() ||
         traj_controller_->has_effort_command_interface()))
      {
        // expect: error.velocity = desired.velocity - current.velocity;
        EXPECT_NEAR(error[i].velocity, desired[i].velocity - current[i].velocity, EPS);
      }
      if (
        traj_controller_->has_acceleration_state_interface() &&
        traj_controller_->has_acceleration_command_interface())
      {
        // expect: error.acceleration = desired.acceleration - current.acceleration;
        EXPECT_NEAR(error[i].acceleration, desired[i].acceleration - current[i].acceleration, EPS);
      }
    }
  }

  executor.cancel();
}

/**
 * @brief check if position error of revolute axes aren't wrapped around (state topic)
 */
TEST_P(TrajectoryControllerTestParameterized, position_error_not_angle_wraparound)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  constexpr double k_p = 10.0;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(
    executor, params, true, k_p, 0.0, INITIAL_POS_AXES, INITIAL_VEL_AXES, INITIAL_ACC_AXES,
    INITIAL_EFF_AXES, test_mttc::urdf_rrrbot_revolute);
  subscribeToState(executor);

  std::size_t n_axes = axis_names_.size();

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

  // does the message have the correct sizes?
  ASSERT_EQ(n_axes, state_reference.size());
  ASSERT_EQ(n_axes, state_feedback.size());
  ASSERT_EQ(n_axes, state_error.size());

  for (std::size_t i = 0; i < n_axes; ++i)
  {
    // no update of state_interface
    EXPECT_EQ(state_feedback[i].position, INITIAL_POS_AXES[i]);
  }

  // are the correct reference position used?
  EXPECT_NEAR(points[0][0], state_reference[0].position, COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][1], state_reference[1].position, COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][2], state_reference[2].position, COMMON_THRESHOLD);

  // no normalization of position error
  EXPECT_NEAR(state_error[0].position, state_reference[0].position - INITIAL_POS_AXES[0], EPS);
  EXPECT_NEAR(state_error[1].position, state_reference[1].position - INITIAL_POS_AXES[1], EPS);
  EXPECT_NEAR(state_error[2].position, state_reference[2].position - INITIAL_POS_AXES[2], EPS);

  if (traj_controller_->has_position_command_interface())
  {
    // check command interface
    EXPECT_NEAR(points[0][0], axis_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][1], axis_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][2], axis_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // use_closed_loop_pid_adapter_
    if (traj_controller_->use_closed_loop_pid_adapter())
    {
      // we expect u = k_p * (s_d-s) for position
      EXPECT_NEAR(
        k_p * (state_reference[0].position - INITIAL_POS_AXES[0]), axis_vel_[0],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference[1].position - INITIAL_POS_AXES[1]), axis_vel_[1],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference[2].position - INITIAL_POS_AXES[2]), axis_vel_[2],
        k_p * COMMON_THRESHOLD);
    }
    else
    {
      // interpolated points_velocity only
      // check command interface
      EXPECT_LT(0.0, axis_vel_[0]);
      EXPECT_LT(0.0, axis_vel_[1]);
      EXPECT_LT(0.0, axis_vel_[2]);
    }
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // with effort command interface, use_closed_loop_pid_adapter is always true
    // we expect u = k_p * (s_d-s) for position
    EXPECT_NEAR(
      k_p * (state_reference[0].position - INITIAL_POS_AXES[0]), axis_eff_[0],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference[1].position - INITIAL_POS_AXES[1]), axis_eff_[1],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference[2].position - INITIAL_POS_AXES[2]), axis_eff_[2],
      k_p * COMMON_THRESHOLD);
  }

  executor.cancel();
}

/**
 * @brief check if position error of continuous axes are wrapped around (state topic)
 */
TEST_P(TrajectoryControllerTestParameterized, position_error_angle_wraparound)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  constexpr double k_p = 10.0;
  std::vector<rclcpp::Parameter> params = {};
  SetUpAndActivateTrajectoryController(
    executor, params, true, k_p, 0.0, INITIAL_POS_AXES, INITIAL_VEL_AXES, INITIAL_ACC_AXES,
    INITIAL_EFF_AXES, test_mttc::urdf_rrrbot_continuous);

  std::size_t n_axes = axis_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // no update of state_interface
  for (std::size_t axis = 0; axis < n_axes; ++axis)
  {
    EXPECT_EQ(state_feedback[axis].position, INITIAL_POS_AXES[axis]);
  }

  // has the msg the correct vector sizes?
  ASSERT_EQ(n_axes, state_reference.size());
  ASSERT_EQ(n_axes, state_feedback.size());
  ASSERT_EQ(n_axes, state_error.size());

  // are the correct reference position used?
  EXPECT_NEAR(points[0][0], state_reference[0].position, COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][1], state_reference[1].position, COMMON_THRESHOLD);
  EXPECT_NEAR(points[0][2], state_reference[2].position, COMMON_THRESHOLD);

  // is error.position[2] wrapped around?
  EXPECT_NEAR(state_error[0].position, state_reference[0].position - INITIAL_POS_AXES[0], EPS);
  EXPECT_NEAR(state_error[1].position, state_reference[1].position - INITIAL_POS_AXES[1], EPS);
  EXPECT_NEAR(
    state_error[2].position, state_reference[2].position - INITIAL_POS_AXES[2] - 2 * M_PI, EPS);

  if (traj_controller_->has_position_command_interface())
  {
    // check command interface
    EXPECT_NEAR(points[0][0], axis_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][1], axis_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points[0][2], axis_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // use_closed_loop_pid_adapter_
    if (traj_controller_->use_closed_loop_pid_adapter())
    {
      // we expect u = k_p * (s_d-s) for axis0 and axis1
      EXPECT_NEAR(
        k_p * (state_reference[0].position - INITIAL_POS_AXES[0]), axis_vel_[0],
        k_p * COMMON_THRESHOLD);
      EXPECT_NEAR(
        k_p * (state_reference[1].position - INITIAL_POS_AXES[1]), axis_vel_[1],
        k_p * COMMON_THRESHOLD);
      // is error of position[2] wrapped around?
      EXPECT_GT(0.0, axis_vel_[2]);  // direction change because of angle wrap
      EXPECT_NEAR(
        k_p * (state_reference[2].position - INITIAL_POS_AXES[2] - 2 * M_PI), axis_vel_[2],
        k_p * COMMON_THRESHOLD);
    }
    else
    {
      // interpolated points_velocity only
      // check command interface
      EXPECT_LT(0.0, axis_vel_[0]);
      EXPECT_LT(0.0, axis_vel_[1]);
      EXPECT_LT(0.0, axis_vel_[2]);
    }
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // with effort command interface, use_closed_loop_pid_adapter is always true
    // we expect u = k_p * (s_d-s) for axis0 and axis1
    EXPECT_NEAR(
      k_p * (state_reference[0].position - INITIAL_POS_AXES[0]), axis_eff_[0],
      k_p * COMMON_THRESHOLD);
    EXPECT_NEAR(
      k_p * (state_reference[1].position - INITIAL_POS_AXES[1]), axis_eff_[1],
      k_p * COMMON_THRESHOLD);
    // is error of position[2] wrapped around?
    EXPECT_GT(0.0, axis_eff_[2]);
    EXPECT_NEAR(
      k_p * (state_reference[2].position - INITIAL_POS_AXES[2] - 2 * M_PI), axis_eff_[2],
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

  std::size_t n_axes = axis_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 4);

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_axes, state_reference.size());

  // is the trajectory still active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should still hold the points from above
  for (std::size_t i = 0; i < n_axes; ++i)
  {
    EXPECT_TRUE(traj_controller_->has_nontrivial_traj(i));
  }

  EXPECT_NEAR(state_reference[0].position, points.at(2).at(0), 1e-2);
  EXPECT_NEAR(state_reference[1].position, points.at(2).at(1), 1e-2);
  EXPECT_NEAR(state_reference[2].position, points.at(2).at(2), 1e-2);
  // value of velocity is different from above due to spline interpolation
  EXPECT_GT(state_reference[0].velocity, 0.0);
  EXPECT_GT(state_reference[1].velocity, 0.0);
  EXPECT_GT(state_reference[2].velocity, 0.0);

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
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*

  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  // update until end of trajectory -> no timeout should have occurred
  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 3);
  // is a trajectory active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should have the trajectory with three points
  std::size_t n_axes = axis_names_.size();
  for (std::size_t i = 0; i < n_axes; ++i)
  {
    EXPECT_TRUE(traj_controller_->has_nontrivial_traj(i));
  }

  // update until timeout should have happened
  updateController(rclcpp::Duration(FIRST_POINT_TIME));

  // after timeout, set_hold_position adds a new trajectory
  // is a trajectory active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should be not more than one point now (from hold position)
  for (std::size_t i = 0; i < n_axes; ++i)
  {
    EXPECT_FALSE(traj_controller_->has_nontrivial_traj(i));
  }
  // should hold last position with zero velocity
  if (traj_controller_->has_position_command_interface())
  {
    expectCommandPoint(points.at(2));
  }
  else
  {
    // no integration to position state interface from velocity/acceleration
    expectCommandPoint(INITIAL_POS_AXES);
  }

  executor.cancel();
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

  std::size_t n_axes = axis_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points_position{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.1, 0.1, 0.1}}, {{0.2, 0.2, 0.2}}, {{0.3, 0.3, 0.3}}};
  // *INDENT-ON*
  publish(time_from_start, points_position, rclcpp::Time(), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_axes, state_reference.size());
  EXPECT_EQ(n_axes, state_feedback.size());
  EXPECT_EQ(n_axes, state_error.size());
  if (traj_controller_->has_velocity_state_interface())
  {
    EXPECT_EQ(n_axes, state_reference.size());
    EXPECT_EQ(n_axes, state_feedback.size());
    EXPECT_EQ(n_axes, state_error.size());
  }

  // no change in state interface should happen
  if (traj_controller_->has_velocity_state_interface())
  {
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      EXPECT_EQ(state_feedback[i].velocity, INITIAL_VEL_AXES[i]);
    }
  }
  // is the velocity error correct?
  if (
    traj_controller_->use_closed_loop_pid_adapter()  // always needed for PID controller
    || (traj_controller_->has_velocity_state_interface() &&
        traj_controller_->has_velocity_command_interface()))
  {
    // don't check against a value, because spline interpolation might overshoot depending on
    // interface combinations
    EXPECT_GE(state_error[0].velocity, points_velocity[0][0]);
    EXPECT_GE(state_error[1].velocity, points_velocity[0][1]);
    EXPECT_GE(state_error[2].velocity, points_velocity[0][2]);
  }

  executor.cancel();
}

/**
 * @brief test_jumbled_axis_order Test sending trajectories with a axis order different from
 * internal controller order
 */
TEST_P(TrajectoryControllerTestParameterized, test_jumbled_axis_order)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);
  std::vector<double> points_position = {1.0, 2.0, 3.0};
  std::vector<std::size_t> jumble_map = {1, 2, 0};
  double dt = 0.25;
  {
    control_msgs::msg::MultiAxisTrajectory traj_msg;
    const std::vector<std::string> jumbled_axis_names{
      axis_names_[jumble_map[0]], axis_names_[jumble_map[1]], axis_names_[jumble_map[2]]};

    traj_msg.axis_names = jumbled_axis_names;
    traj_msg.header.stamp = rclcpp::Time(0);

    std::size_t n_axes = axis_names_.size();
    traj_msg.axis_trajectories.resize(n_axes);
    for (std::size_t i = 0; i < n_axes; ++i)
    {
      traj_msg.axis_trajectories[i].axis_points.resize(1);
      traj_msg.axis_trajectories[i].axis_points[0].time_from_start =
        rclcpp::Duration::from_seconds(dt);
      traj_msg.axis_trajectories[i].axis_points[0].position = points_position.at(jumble_map[i]);
      traj_msg.axis_trajectories[i].axis_points[0].velocity =
        (traj_msg.axis_trajectories[i].axis_points[0].position - axis_pos_[jumble_map[i]]) / dt;
      traj_msg.axis_trajectories[i].axis_points[0].acceleration =
        (traj_msg.axis_trajectories[i].axis_points[0].velocity - axis_vel_[jumble_map[i]]) / dt;
    }

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(dt));

  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(points_position.at(0), axis_pos_[0], COMMON_THRESHOLD);
    EXPECT_NEAR(points_position.at(1), axis_pos_[1], COMMON_THRESHOLD);
    EXPECT_NEAR(points_position.at(2), axis_pos_[2], COMMON_THRESHOLD);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    // if use_closed_loop_pid_adapter_==false: we expect desired velocity from direct sampling
    // if use_closed_loop_pid_adapter_==true: we expect desired velocity, because we use PID with
    // feedforward term only
    EXPECT_GT(0.0, axis_vel_[0]);
    EXPECT_GT(0.0, axis_vel_[1]);
    EXPECT_GT(0.0, axis_vel_[2]);
  }

  if (traj_controller_->has_acceleration_command_interface())
  {
    EXPECT_GT(0.0, axis_acc_[0]);
    EXPECT_GT(0.0, axis_acc_[1]);
    EXPECT_GT(0.0, axis_acc_[2]);
  }

  if (traj_controller_->has_effort_command_interface())
  {
    // effort should be nonzero, because we use PID with feedforward term
    EXPECT_GT(0.0, axis_eff_[0]);
    EXPECT_GT(0.0, axis_eff_[1]);
    EXPECT_GT(0.0, axis_eff_[2]);
  }
}

/**
 * @brief invalid_message Test mismatched axis and reference vector sizes
 */
TEST_P(TrajectoryControllerTestParameterized, invalid_message)
{
  rclcpp::Parameter allow_integration_parameters("allow_integration_in_goal_trajectories", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {allow_integration_parameters});

  control_msgs::msg::MultiAxisTrajectory traj_msg, good_traj_msg;

  good_traj_msg.axis_names = axis_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  std::size_t const num_axes = 3;
  good_traj_msg.axis_trajectories.resize(num_axes);

  std::vector<double> const positions = {1.0, 2.0, 3.0};
  std::vector<double> const velocities = {-1.0, -2.0, -3.0};
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    good_traj_msg.axis_trajectories[i].axis_points.resize(1);
    good_traj_msg.axis_trajectories[i].axis_points[0].time_from_start =
      rclcpp::Duration::from_seconds(0.25);
    good_traj_msg.axis_trajectories[i].axis_points[0].position = positions[i];
    good_traj_msg.axis_trajectories[i].axis_points[0].velocity = velocities[i];
  }
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // Incompatible axis names
  traj_msg = good_traj_msg;
  traj_msg.axis_names = {"bad_name"};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // empty message
  traj_msg = good_traj_msg;
  traj_msg.axis_trajectories.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points[0].position =
      std::numeric_limits<double>::quiet_NaN();
  }
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Non-monotonically increasing waypoint times
  traj_msg = good_traj_msg;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points.push_back(
      traj_msg.axis_trajectories[i].axis_points.front());
  }
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
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

  control_msgs::msg::MultiAxisTrajectory traj_msg;
  traj_msg.axis_names = axis_names_;
  traj_msg.header.stamp = rclcpp::Time(0);

  // empty message (no throw!)
  ASSERT_NO_THROW(traj_controller_->validate_trajectory_msg(traj_msg));
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Nonzero velocity at trajectory end!
  std::size_t num_axes = axis_names_.size();
  traj_msg.axis_trajectories.resize(num_axes);
  std::vector<double> const positions = {1.0, 2.0, 3.0};
  std::vector<double> const velocities = {-1.0, -2.0, -3.0};
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points.resize(1);
    traj_msg.axis_trajectories[i].axis_points[0].time_from_start =
      rclcpp::Duration::from_seconds(0.25);
    traj_msg.axis_trajectories[i].axis_points[0].position = positions[i];
    traj_msg.axis_trajectories[i].axis_points[0].velocity = velocities[i];
  }
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief missing_position_message_accepted Test mismatched axis and reference vector sizes
 *
 * @note With allow_integration_in_goal_trajectories parameter trajectory missing position or
 * velocity are accepted
 */
TEST_P(TrajectoryControllerTestParameterized, missing_position_message_accepted)
{
  rclcpp::Parameter allow_integration_parameters("allow_integration_in_goal_trajectories", true);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {allow_integration_parameters});

  control_msgs::msg::MultiAxisTrajectory traj_msg, good_traj_msg;

  good_traj_msg.axis_names = axis_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  std::size_t const num_axes = axis_names_.size();
  good_traj_msg.axis_trajectories.resize(num_axes);
  std::vector<double> const positions = {1.0, 2.0, 3.0};
  std::vector<double> const velocities = {-1.0, -2.0, -3.0};
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    good_traj_msg.axis_trajectories[i].axis_points.resize(1);
    good_traj_msg.axis_trajectories[i].axis_points[0].time_from_start =
      rclcpp::Duration::from_seconds(0.25);
    good_traj_msg.axis_trajectories[i].axis_points[0].position = positions[i];
    good_traj_msg.axis_trajectories[i].axis_points[0].velocity = velocities[i];
  }
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points[0].position =
      std::numeric_limits<double>::quiet_NaN();
  }
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position and velocity data
  traj_msg = good_traj_msg;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points[0].position =
      std::numeric_limits<double>::quiet_NaN();
    traj_msg.axis_trajectories[i].axis_points[0].velocity =
      std::numeric_limits<double>::quiet_NaN();
  }
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

  // All empty
  traj_msg = good_traj_msg;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    traj_msg.axis_trajectories[i].axis_points[0].position =
      std::numeric_limits<double>::quiet_NaN();
    traj_msg.axis_trajectories[i].axis_points[0].velocity =
      std::numeric_limits<double>::quiet_NaN();
    traj_msg.axis_trajectories[i].axis_points[0].acceleration =
      std::numeric_limits<double>::quiet_NaN();
  }
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief test_trajectory_replace Test replacing an existing trajectory
 */
TEST_P(TrajectoryControllerTestParameterized, test_trajectory_replace)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  std::vector<std::vector<double>> positions_old{{2., 3., 4.}};
  std::vector<std::vector<double>> velocities_old{{0.2, 0.3, 0.4}};
  std::vector<std::vector<double>> positions_new_partial{{1.5}};
  std::vector<std::vector<double>> velocities_new_partial{{0.15}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, positions_old, rclcpp::Time(), {}, velocities_old);
  std::vector<control_msgs::msg::AxisTrajectoryPoint> expected_actual, expected_desired;
  std::size_t num_axes = positions_old[0].size();
  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());
  expected_desired.resize(num_axes);
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = positions_old[0][i];
    expected_desired[i].position = positions_old[0][i];
    expected_actual[i].velocity = velocities_old[0][i];
    expected_desired[i].velocity = velocities_old[0][i];
  }
  //  Check that we reached end of points_old trajectory
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory");
  velocities_new_partial[0][0] =
    std::copysign(0.15, positions_new_partial[0][0] - axis_state_pos_[0]);
  publish(time_from_start, positions_new_partial, rclcpp::Time(), {}, velocities_new_partial);

  // Replaced trajectory is a mix of previous and current goal
  expected_desired[0].position = positions_new_partial[0][0];
  expected_desired[1].position = positions_old[0][1];
  expected_desired[2].position = positions_old[0][2];
  expected_desired[0].velocity = velocities_new_partial[0][0];
  expected_desired[1].velocity = 0.0;
  expected_desired[2].velocity = 0.0;
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

  // TODO(anyone): add expectations for velocity and acceleration
  std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new{{{-1., -2., -3.}}};

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the future");
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old, rclcpp::Time());

  // a vector of axis states at each time
  std::vector<control_msgs::msg::AxisTrajectoryPoint> expected_actuals, expected_desireds;
  std::size_t num_axes = points_old[0].size();
  expected_actuals.resize(num_axes);
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actuals[i].position = points_old[0][i];
  }
  expected_desireds = expected_actuals;
  //  Check that we reached end of points_old[0] trajectory
  auto end_time = waitAndCompareState(
    expected_actuals, expected_desireds, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the past");
  //  New trajectory will end before current time
  rclcpp::Time new_traj_start =
    rclcpp::Clock(RCL_STEADY_TIME).now() - delay - std::chrono::milliseconds(100);
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actuals[i].position = points_old[1][i];
  }
  expected_desireds = expected_actuals;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(
    expected_actuals, expected_desireds, executor, rclcpp::Duration(delay), 0.1, end_time);
}

TEST_P(TrajectoryControllerTestParameterized, test_ignore_partial_old_trajectory)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new{{{-1., -2., -3.}, {-2., -4., -6.}}};
  std::vector<control_msgs::msg::AxisTrajectoryPoint> expected_actual, expected_desired;
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};

  // send points_old and wait to reach first point
  publish(time_from_start, points_old, rclcpp::Time());
  std::size_t num_axes = points_old.size();
  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = points_old[0][i];
  }
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
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = points_old[1][i];
  }
  expected_desired = expected_actual;
  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1, end_time);
}

TEST_P(TrajectoryControllerTestParameterized, test_execute_partial_traj_in_future)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, {});

  RCLCPP_WARN(
    traj_controller_->get_node()->get_logger(),
    "Test disabled until current_trajectory is taken into account when adding a new trajectory.");
  return;

  // *INDENT-OFF*
  std::vector<std::vector<double>> full_traj{{{2., 3., 4.}, {4., 6., 8.}}};
  std::vector<std::vector<double>> full_traj_velocity{{{0.2, 0.3, 0.4}, {0.4, 0.6, 0.8}}};
  std::vector<std::vector<double>> partial_traj{{{-1., -2.}, {-2., -4}}};
  std::vector<std::vector<double>> partial_traj_velocity{{{-0.1, -0.2}, {-0.2, -0.4}}};
  // *INDENT-ON*
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration points_delay{rclcpp::Duration(delay)};
  // Send full trajectory
  publish(points_delay, full_traj, rclcpp::Time(), {}, full_traj_velocity);
  // Sleep until first waypoint of full trajectory

  std::vector<control_msgs::msg::AxisTrajectoryPoint> expected_actual, expected_desired;
  std::size_t num_axes = full_traj[0].size();
  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = full_traj[0][i];
  }
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory and are starting points_old[1]
  auto end_time =
    waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  // Send partial trajectory starting after full trajecotry is complete
  RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the future");
  publish(
    points_delay, partial_traj, rclcpp::Clock(RCL_STEADY_TIME).now() + delay * 2, {},
    partial_traj_velocity);
  // Wait until the end start and end of partial traj

  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = partial_traj.back()[i];
  }
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
  std::vector<std::vector<double>> first_goal_velocity = {{0.33, 0.44, 0.55}};
  std::vector<double> second_goal = {6.6, 8.8, 11.0};
  std::vector<std::vector<double>> second_goal_velocity = {{0.66, 0.88, 1.1}};
  double state_from_command_offset = 0.3;

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  double trajectory_frac = rclcpp::Duration::from_seconds(0.01).seconds() /
                           (time_from_start.sec + time_from_start.nanosec * 1e-9);
  std::vector<std::vector<double>> points{{first_goal}};
  publish(
    time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME), {}, first_goal_velocity);
  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1));

  // JTC is executing trajectory in open-loop therefore:
  // - internal state does not have to be updated (in this test-case it shouldn't)
  // - internal command is updated
  EXPECT_NEAR(INITIAL_POS_AXIS1, axis_state_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  axis_state_pos_[0] = first_goal[0] - state_from_command_offset;

  // Move axis further in the same direction as before (to the second goal)
  points = {{second_goal}};
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, second_goal_velocity);
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there should be a "jump" in opposite direction from command
  // (towards the state value)
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);
  auto end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // Expect backward commands at first, consider advancement of the trajectory
  // exact value is not directly predictable, because of the spline interpolation -> increase
  // tolerance
  EXPECT_NEAR(
    axis_state_pos_[0] + (second_goal[0] - axis_state_pos_[0]) * trajectory_frac, axis_pos_[0],
    0.1);
  EXPECT_GT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_LT(axis_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_LT(axis_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_LT(axis_pos_[0], first_goal[0]);

  // Finally the second goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(second_goal[0], axis_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  axis_state_pos_[0] = second_goal[0] - state_from_command_offset;

  // Move axis back to the first goal
  points = {{first_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there should be a "jump" in the goal direction from command
  // (towards the state value)
  EXPECT_NEAR(second_goal[0], axis_pos_[0], COMMON_THRESHOLD);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // Expect backward commands at first, consider advancement of the trajectory
  EXPECT_NEAR(
    axis_state_pos_[0] + (first_goal[0] - axis_state_pos_[0]) * trajectory_frac, axis_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_LT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_LT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_LT(axis_pos_[0], axis_state_pos_[0]);
  EXPECT_GT(axis_pos_[0], first_goal[0]);

  // Finally the first goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);

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
  EXPECT_NEAR(INITIAL_POS_AXIS1, axis_state_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  axis_state_pos_[0] = first_goal[0] - state_from_command_offset;

  // Move axis further in the same direction as before (to the second goal)
  points = {{second_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there **should not** be a "jump" in opposite direction from
  // command (towards the state value)
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);
  auto end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01));
  // There should not be backward commands
  EXPECT_NEAR(
    first_goal[0] + (second_goal[0] - first_goal[0]) * trajectory_frac, axis_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  EXPECT_LT(axis_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  EXPECT_LT(axis_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  EXPECT_LT(axis_pos_[0], second_goal[0]);

  // Finally the second goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(second_goal[0], axis_pos_[0], COMMON_THRESHOLD);

  // State interface should have offset from the command before starting a new trajectory
  axis_state_pos_[0] = second_goal[0] - state_from_command_offset;

  // Move axis back to the first goal
  points = {{first_goal}};
  publish(time_from_start, points, rclcpp::Time(0.0, 0.0, RCL_STEADY_TIME));
  traj_controller_->wait_for_trajectory(executor);

  // One the first update(s) there **should not** be a "jump" in the goal direction from
  // command (towards the state value)
  EXPECT_NEAR(second_goal[0], axis_pos_[0], COMMON_THRESHOLD);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  // There should not be a jump toward commands
  EXPECT_NEAR(
    second_goal[0] + (first_goal[0] - second_goal[0]) * trajectory_frac, axis_pos_[0],
    COMMON_THRESHOLD);
  EXPECT_LT(axis_pos_[0], second_goal[0]);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  EXPECT_LT(axis_pos_[0], second_goal[0]);
  end_time = updateControllerAsync(rclcpp::Duration::from_seconds(0.01), end_time);
  EXPECT_GT(axis_pos_[0], first_goal[0]);
  EXPECT_LT(axis_pos_[0], second_goal[0]);

  // Finally the first goal will be commanded/reached
  updateControllerAsync(rclcpp::Duration::from_seconds(1.1), end_time);
  EXPECT_NEAR(first_goal[0], axis_pos_[0], COMMON_THRESHOLD);

  executor.cancel();
}

// Testing that values are read from state interfaces when hardware is started for the first
// time and hardware state has offset --> this is indicated by NaN values in command interfaces
TEST_P(TrajectoryControllerTestParameterized, test_hw_states_has_offset_first_controller_start)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);

  // set command values to NaN
  std::vector<double> initial_pos_cmd{
    3, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> initial_vel_cmd{
    3, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> initial_acc_cmd{
    3, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};

  SetUpAndActivateTrajectoryController(
    executor, {is_open_loop_parameters}, true, 0., 1., initial_pos_cmd, initial_vel_cmd,
    initial_acc_cmd);

  // no call of update method, so the values should be read from state interfaces
  // (command interface are NaN)

  auto current_state_when_offset = traj_controller_->get_current_state_when_offset();
  std::size_t num_axes = initial_pos_cmd.size();

  for (std::size_t i = 0; i < num_axes; ++i)
  {
    EXPECT_EQ(current_state_when_offset[i].position, axis_state_pos_[i]);

    // check velocity
    if (traj_controller_->has_velocity_state_interface())
    {
      EXPECT_EQ(current_state_when_offset[i].velocity, axis_state_vel_[i]);
    }

    // check acceleration
    if (traj_controller_->has_acceleration_state_interface())
    {
      EXPECT_EQ(current_state_when_offset[i].acceleration, axis_state_acc_[i]);
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
  std::size_t num_axes = 3;
  for (std::size_t i = 0; i < num_axes; ++i)
  {
    initial_pos_cmd.push_back(3.1 + static_cast<double>(i));
    initial_vel_cmd.push_back(0.25 + static_cast<double>(i));
    initial_acc_cmd.push_back(0.02 + static_cast<double>(i) / 10.0);
  }
  SetUpAndActivateTrajectoryController(
    executor, {is_open_loop_parameters}, true, 0., 1., initial_pos_cmd, initial_vel_cmd,
    initial_acc_cmd);

  // no call of update method, so the values should be read from command interfaces

  auto current_state_when_offset = traj_controller_->get_current_state_when_offset();
  for (std::size_t i = 0; i < num_axes; ++i)
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
              EXPECT_EQ(current_state_when_offset[i].position, initial_pos_cmd[i]);
              EXPECT_EQ(current_state_when_offset[i].velocity, initial_vel_cmd[i]);
              EXPECT_EQ(current_state_when_offset[i].acceleration, initial_acc_cmd[i]);
            }
            else
            {
              // should have set it to the state interface instead
              EXPECT_EQ(current_state_when_offset[i].position, axis_state_pos_[i]);
              EXPECT_EQ(current_state_when_offset[i].velocity, axis_state_vel_[i]);
              EXPECT_EQ(current_state_when_offset[i].acceleration, axis_state_acc_[i]);
            }
          }
          else
          {
            // should have set it to last position + velocity command
            EXPECT_EQ(current_state_when_offset[i].position, initial_pos_cmd[i]);
            EXPECT_EQ(current_state_when_offset[i].velocity, initial_vel_cmd[i]);
          }
        }
        else
        {
          // should have set it to the state interface instead
          EXPECT_EQ(current_state_when_offset[i].position, axis_state_pos_[i]);
          EXPECT_EQ(current_state_when_offset[i].velocity, axis_state_vel_[i]);
        }
      }
      else
      {
        // should have set it to last position command
        EXPECT_EQ(current_state_when_offset[i].position, initial_pos_cmd[i]);
      }
    }
    else
    {
      // should have set it to the state interface instead
      EXPECT_EQ(current_state_when_offset[i].position, axis_state_pos_[i]);
    }
  }

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, test_state_tolerances_fail)
{
  // set axis tolerance parameters
  const double state_tol = 0.0001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.axis1.trajectory", state_tol),
    rclcpp::Parameter("constraints.axis2.trajectory", state_tol),
    rclcpp::Parameter("constraints.axis3.trajectory", state_tol)};

  rclcpp::executors::MultiThreadedExecutor executor;
  double kp = 1.0;  // activate feedback control for testing velocity/effort PID
  SetUpAndActivateTrajectoryController(executor, params, true, kp);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(100);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);
  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME));

  // it should have aborted and be holding now
  expectCommandPoint(axis_state_pos_);
}

TEST_P(TrajectoryControllerTestParameterized, test_goal_tolerances_fail)
{
  // set axis tolerance parameters
  const double goal_tol = 0.1;
  // set very small goal_time so that goal_time is violated
  const double goal_time = 0.000001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.axis1.goal", goal_tol),
    rclcpp::Parameter("constraints.axis2.goal", goal_tol),
    rclcpp::Parameter("constraints.axis3.goal", goal_tol),
    rclcpp::Parameter("constraints.goal_time", goal_time)};

  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, params, true, 1.0);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(100);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);
  auto end_time = updateControllerAsync(rclcpp::Duration(4 * FIRST_POINT_TIME));

  // it should have aborted and be holding now
  expectCommandPoint(axis_state_pos_);

  // what happens if we wait longer but it harms the tolerance again?
  auto hold_position = axis_state_pos_;
  axis_state_pos_.at(0) = -3.3;
  updateControllerAsync(rclcpp::Duration(FIRST_POINT_TIME), end_time);
  // it should be still holding the old point
  expectCommandPoint(hold_position);
}

TEST_P(TrajectoryControllerTestParameterized, open_closed_enable_disable)
{
  // set up controller into open loop mode
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params = {{"open_loop_control", true}, {"use_feedback", true}};
  SetUpAndActivateTrajectoryController(executor, params, true);

  // [axis-mult] At 20 Hz, sends a 'reference' command with all zeros and time from start of 50ms
  // (i.e. positions are NaN, velocities are zero and accelerations are NaN)

  constexpr std::size_t freq_Hz = 20;
  std::size_t const num_axes = 3;
  std::size_t const ns_dur = 1000000000 / freq_Hz;
  auto const chrono_duration = std::chrono::nanoseconds(ns_dur);
  rclcpp::Duration const dur(0, ns_dur);
  double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> point_nan = {nan, nan, nan};
  std::vector<double> point_zero = {0, 0, 0};

  // start with zero vels and nan positions for 1 second
  std::vector<std::vector<double>> positions(freq_Hz, point_nan);
  std::vector<std::vector<double>> velocities(freq_Hz, point_zero);

  std::vector<control_msgs::msg::AxisTrajectoryPoint> expected_actual, expected_desired;
  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());

  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = 0;
    expected_actual[i].velocity = 0;
  }

  expected_desired = expected_actual;

  publish(dur, positions, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, velocities);
  positions.clear();
  velocities.clear();
  traj_controller_->wait_for_trajectory(executor);

  // now test that we haven't moved
  waitAndCompareState(expected_actual, expected_desired, executor, chrono_duration * freq_Hz, 0.1);
  expected_actual.clear();
  expected_desired.clear();

  // [axis-mult] When joystick is moved, it sends 'reference' command with non-zero velocities for
  // axes moved in joystick. As we are in open-loop teleoperation, this command works

  positions = {freq_Hz, point_nan};

  // 0.5 second of constant accel
  for (std::size_t i = 0; i < freq_Hz / 2; ++i)
  {
    double target_vel_current = static_cast<double>(i) * static_cast<double>(i);
    // each axis's target velocity is proportional to time ^ 2, which should give a constant accel
    velocities.push_back(
      {0.01 * target_vel_current, 0.02 * target_vel_current, 0.03 * target_vel_current});
  }

  // then 0.5 seconds of constant vel
  auto const final_vel = velocities.back();
  for (std::size_t i = 0; i < freq_Hz / 2; ++i)
  {
    velocities.push_back(final_vel);
  }

  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());

  for (std::size_t i = 0; i < num_axes; ++i)
  {
    // the final position should be the integral of the velocity profile we gave
    // for constant accel portion, that is 0.5 * (0.5 seconds) * final_vel
    // for constant vel portion, that is just 0.5 seconds * final_vel
    expected_actual[i].position = 0.5 * 0.5 * final_vel[i] + 0.5 * final_vel[i];
    expected_actual[i].velocity = final_vel[i];
  }

  expected_desired = expected_actual;

  publish(dur, positions, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, velocities);
  positions.clear();
  velocities.clear();
  traj_controller_->wait_for_trajectory(executor);

  waitAndCompareState(expected_actual, expected_desired, executor, chrono_duration * freq_Hz, 0.1);
  expected_actual.clear();
  expected_desired.clear();

  // [axis-mult] Joystick is released and it continually sends a 'reference' command with all zeros
  // and time from start of 50ms(i.e. positions are NaN, velocities are zero and accelerations are
  // NaN)

  positions = {freq_Hz, point_nan};
  velocities = {freq_Hz, point_zero};

  publish(dur, positions, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, velocities);

  // store the final position
  auto final_pos = axis_pos_;

  expected_actual.resize(num_axes, multi_time_trajectory_controller::emptyTrajectoryPoint());

  for (std::size_t i = 0; i < num_axes; ++i)
  {
    expected_actual[i].position = final_pos[i];
    expected_actual[i].velocity = 0;
  }

  positions.clear();
  velocities.clear();
  traj_controller_->wait_for_trajectory(executor);

  waitAndCompareState(expected_actual, expected_desired, executor, chrono_duration * freq_Hz, 0.1);
  expected_actual.clear();
  expected_desired.clear();

  // store the new final position
  final_pos = axis_pos_;

  // [external] Closed-loop control is enabled on X-Y i.e. on position and velocity controllers
  // [axis-mult] Detects closed loop control is enabled for x-y axes
  // [axis-mult] Sends a 'reset dofs' service call to CTG/MAC to reset x-y position to values
  // specified in service call(values are current x-y state estimate)

  // [axis-mult] When service request is completed, sends a 'reference_reliable' command to CTG/MAC
  // with current x-y state estimate with 'time from start' set to 10ms (essentially a command to
  // hold current position to be sent to closed-loop position controller). The velocities are set to
  // zero and the accelerations to NaN in this command.

  // [axis-mult] At 20 Hz, continues sending 'reference' command of zeroes at 50ms

  // [axis-mult] When joystick is moved in X-Y to perform closed-loop teleoperation, it sends
  // 'reference' command with non-zero velocities for axes moved in joystick. This command doesn't
  // work on MAC but does on CTG. If I disable closed-loop control for X-Y, and try to move X-Y in
  // open-loop with joystick again on MAC, it doesn't move. If I move the joystick in a different
  // axes e.g. heading, it works fine. I have to confirm again but Z doesn't work anymore though
  // either on MAC.
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
