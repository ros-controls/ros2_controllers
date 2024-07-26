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

#include "joint_trajectory_controller/trajectory.hpp"

#include <algorithm>
#include <memory>

#include "angles/angles.h"
#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace joint_trajectory_controller
{
namespace
{
// Safe default joint kinematic limits for Ruckig, in case none is defined in the URDF or
// joint_limits.yaml
constexpr double DEFAULT_MAX_VELOCITY = 1.5;      // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 5.0;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 200.0;        // rad/s^3
}  // namespace

Trajectory::Trajectory()
: trajectory_start_time_(0),
  time_before_traj_msg_(0),
  sampled_already_(false),
  have_previous_ruckig_output_(false)
{
}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp)),
  sampled_already_(false),
  have_previous_ruckig_output_(false)
{
}

Trajectory::Trajectory(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{
  set_point_before_trajectory_msg(current_time, current_point);
  //   update(joint_trajectory);
}

void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
  const std::vector<bool> & joints_angle_wraparound)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;

  // Compute offsets due to wrapping joints
  wraparound_joint(
    state_before_traj_msg_.positions, trajectory_msg_->points[0].positions,
    joints_angle_wraparound);
}

void wraparound_joint(
  std::vector<double> & current_position, const std::vector<double> next_position,
  const std::vector<bool> & joints_angle_wraparound)
{
  double dist;
  // joints_angle_wraparound is even empty, or has the same size as the number of joints
  for (size_t i = 0; i < joints_angle_wraparound.size(); i++)
  {
    if (joints_angle_wraparound[i])
    {
      dist = angles::shortest_angular_distance(current_position[i], next_position[i]);

      // Deal with singularity at M_PI shortest distance
      if (std::abs(std::abs(dist) - M_PI) < 1e-9)
      {
        dist = next_position[i] > current_position[i] ? std::abs(dist) : -std::abs(dist);
      }

      current_position[i] = next_position[i] - dist;
    }
  }
}

void Trajectory::update(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory,
  const std::vector<joint_limits::JointLimits> & joint_limits, const rclcpp::Duration & period)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;

  // Initialize Ruckig-smoothing-related stuff
  size_t dim = joint_trajectory->joint_names.size();
  // TODO(andyz): update only the Ruckig::delta_time member of the smoother.
  // dim should not update since it doesn't change with every new trajectory
  // See https://github.com/pantor/ruckig/issues/118
  smoother_ = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(dim, period.seconds());
  ruckig_input_ = ruckig::InputParameter<ruckig::DynamicDOFs>(dim);
  ruckig_output_ = ruckig::OutputParameter<ruckig::DynamicDOFs>(dim);
  have_previous_ruckig_output_ = false;

  ruckig_input_.max_velocity.clear();
  ruckig_input_.max_velocity.resize(dim, DEFAULT_MAX_VELOCITY);
  ruckig_input_.max_acceleration.clear();
  ruckig_input_.max_acceleration.resize(dim, DEFAULT_MAX_ACCELERATION);
  ruckig_input_.max_jerk.clear();
  ruckig_input_.max_jerk.resize(dim, DEFAULT_MAX_JERK);

  for (size_t i = 0; i < dim; ++i)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("trajectory"), "max vel for joint %zu is %f", i,
      ruckig_input_.max_velocity[i]);
    RCLCPP_DEBUG(
      rclcpp::get_logger("trajectory"), "max acc for joint %zu is %f", i,
      ruckig_input_.max_acceleration[i]);
    RCLCPP_DEBUG(
      rclcpp::get_logger("trajectory"), "max jerk for joint %zu is %f", i,
      ruckig_input_.max_jerk[i]);
    if (joint_limits[i].has_velocity_limits)
    {
      ruckig_input_.max_velocity[i] = joint_limits[i].max_velocity;
      RCLCPP_DEBUG(
        rclcpp::get_logger("trajectory"), "Setting max vel for joint %zu to %f", i,
        joint_limits[i].max_velocity);
    }
    if (joint_limits[i].has_acceleration_limits)
    {
      ruckig_input_.max_acceleration[i] = joint_limits[i].max_acceleration;
      RCLCPP_DEBUG(
        rclcpp::get_logger("trajectory"), "Setting max acc for joint %zu to %f", i,
        joint_limits[i].max_acceleration);
    }
    if (joint_limits[i].has_jerk_limits)
    {
      ruckig_input_.max_jerk[i] = joint_limits[i].max_jerk;
      RCLCPP_DEBUG(
        rclcpp::get_logger("trajectory"), "Setting max jerk for joint %zu to %f", i,
        joint_limits[i].max_jerk);
    }
  }

  previous_state_.positions.resize(dim, std::numeric_limits<double>::quiet_NaN());
  previous_state_.velocities.resize(dim, std::numeric_limits<double>::quiet_NaN());
  previous_state_.accelerations.resize(dim, std::numeric_limits<double>::quiet_NaN());
}

bool Trajectory::sample(
  const rclcpp::Time & sample_time,
  const interpolation_methods::InterpolationMethod interpolation_method,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state,
  TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr,
  const rclcpp::Duration & period,
  std::unique_ptr<joint_limits::JointLimiterInterface<joint_limits::JointLimits>> & joint_limiter,
  trajectory_msgs::msg::JointTrajectoryPoint & splines_state,
  trajectory_msgs::msg::JointTrajectoryPoint & ruckig_state,
  trajectory_msgs::msg::JointTrajectoryPoint & ruckig_input_state)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  // first sampling of this trajectory
  if (!sampled_already_)
  {
    if (trajectory_start_time_.seconds() == 0.0)
    {
      trajectory_start_time_ = sample_time;
    }

    sampled_already_ = true;
  }

  // sampling before the current point
  if (sample_time < time_before_traj_msg_)
  {
    return false;
  }

  // TODO(anyone): this shouldn't be initialized at runtime
  output_state = trajectory_msgs::msg::JointTrajectoryPoint();

  auto do_ruckig_smoothing =
    interpolation_method == interpolation_methods::InterpolationMethod::RUCKIG;
  auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  // current time hasn't reached traj time of the first point in the msg yet
  if (sample_time < first_point_timestamp)
  {
    // If interpolation is disabled, just forward the next waypoint
    if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
    {
      output_state = state_before_traj_msg_;
      previous_state_ = state_before_traj_msg_;
    }
    else
    {
      // const size_t dim = first_point_in_msg.velocities.size();
      //
      // double max_vel_ratio = 1.0;
      // for (size_t dim_i = 0; dim_i < dim; ++dim_i)
      // {
      //   if (std::fabs(first_point_in_msg.velocities[dim_i]) > joint_limits[dim_i].max_velocity)
      //   {
      //     const double ratio =
      //     std::fabs(first_point_in_msg.velocities[dim_i] / joint_limits[dim_i].max_velocity);
      //     if (ratio > max_vel_ratio)
      //     {
      //       max_vel_ratio = ratio;
      //     }
      //   }
      // }
      //
      // for (size_t dim_i = 0; dim_i < dim; ++dim_i)
      // {
      //   // Set the target velocities to follow the joint limits
      // first_point_in_msg.velocities[dim_i] = first_point_in_msg.velocities[dim_i] / max_vel_ratio
      // }

      // it changes points only if position and velocity do not exist, but their derivatives
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        (first_point_timestamp - time_before_traj_msg_).seconds());

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time, do_ruckig_smoothing, false, output_state, period, splines_state, ruckig_state,
        ruckig_input_state);
    }
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();

    if (joint_limiter)
    {
      joint_limiter->enforce(previous_state_, output_state, period);
    }
    previous_state_ = output_state;
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto point_before_last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = 0; i < point_before_last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // TODO(anyone): this shouldn't be initialized at runtime
      output_state = trajectory_msgs::msg::JointTrajectoryPoint();

      // If interpolation is disabled, just forward the next waypoint
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      // Do interpolation
      else
      {
        // const size_t dim = next_point.velocities.size();
        //
        // double max_vel_ratio = 1.0;
        // for (size_t dim_i = 0; dim_i < dim; ++dim_i)
        // {
        //   if (std::fabs(next_point.velocities[dim_i]) > joint_limits[dim_i].max_velocity)
        //   {
        //     const double ratio =
        //       std::fabs(next_point.velocities[dim_i] / joint_limits[dim_i].max_velocity);
        //     if (ratio > max_vel_ratio)
        //     {
        //       max_vel_ratio = ratio;
        //     }
        //   }
        // }
        //
        // for (size_t dim_i = 0; dim_i < dim; ++dim_i)
        // {
        //   // Set the target velocities to follow the joint limits
        //   next_point.velocities[dim_i] = next_point.velocities[dim_i] / max_vel_ratio;
        // }

        // it changes points only if position and velocity do not exist, but their derivatives
        deduce_from_derivatives(point, next_point, point.positions.size(), (t1 - t0).seconds());

        if (!interpolate_between_points(
              t0, point, t1, next_point, sample_time, do_ruckig_smoothing, false, output_state,
              period, splines_state, ruckig_state, ruckig_input_state))
        {
          return false;
        }
      }
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);

      if (joint_limiter)
      {
        joint_limiter->enforce(previous_state_, output_state, period);
      }
      previous_state_ = output_state;
      return true;
    }
  }

  // whole animation has played out - but still continue s interpolating and smoothing
  auto & last_point = trajectory_msg_->points[trajectory_msg_->points.size() - 1];
  const rclcpp::Time t0 = trajectory_start_time_ + last_point.time_from_start;

  // FIXME(destogl): this is from backport - check if needed
  //   // whole animation has played out
  //   start_segment_itr = --end();
  //   end_segment_itr = end();
  //   expected_state = (*start_segment_itr);
  //
  //   // TODO: Add and test enforceJointLimits? Unsure if needed for end of animation
  //   // Yes, call enforceJointLimits to handle halting in servo, which has time_from_start ==
  // 1ns (does not enforce vel/acc limits)
  //   if(last_idx == 0) {
  //     // Enforce limits from current state, not the trajectory's single point, because the point
  // from servo halting violates limits
  //     if (joint_limiter)
  //     {
  //       joint_limiter->enforce(
  //         state_before_traj_msg_, expected_state, (sample_time - time_before_traj_msg_));
  //     }
  //   }

  const size_t dim = last_point.positions.size();

  // the trajectories in msg may have empty velocities/accel, so resize them
  if (last_point.velocities.empty())
  {
    last_point.velocities.resize(dim, 0.0);
  }
  if (last_point.accelerations.empty())
  {
    last_point.accelerations.resize(dim, 0.0);
  }

  // integrate velocities and positions because acc and vel don't have to be 0 between points
  for (size_t dim_i = 0; dim_i < dim; ++dim_i)
  {
    if (last_point.accelerations[dim_i] != 0)
    {
      last_point.velocities[dim_i] += last_point.accelerations[dim_i] * period.seconds();
      // remember velocity over multiple calls
    }
    if (last_point.velocities[dim_i] != 0)
    {
      last_point.positions[dim_i] += last_point.velocities[dim_i] * period.seconds();
      // remember velocity over multiple calls
    }
  }

  // do not do splines when trajectory has finished because the time is achieved
  if (!interpolate_between_points(
        t0, last_point, t0, last_point, sample_time, do_ruckig_smoothing, true, output_state,
        period, splines_state, ruckig_state, ruckig_input_state))
  {
    return false;
  }

  if (joint_limiter)
  {
    // TODO(destogl): use here output state
    joint_limiter->enforce(previous_state_, output_state, period);
  }
  previous_state_ = output_state;

  start_segment_itr = --end();
  end_segment_itr = end();

  return true;
}

bool Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time, const bool do_ruckig_smoothing, const bool skip_splines,
  trajectory_msgs::msg::JointTrajectoryPoint & output, const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & splines_state,
  trajectory_msgs::msg::JointTrajectoryPoint & ruckig_state,
  trajectory_msgs::msg::JointTrajectoryPoint & ruckig_input_state)
{
  //   RCLCPP_WARN(rclcpp::get_logger("trajectory"), "New iteration");

  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();

  // TODO(anyone): this shouldn't be resized at runtime
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  splines_state.positions.resize(dim, std::numeric_limits<double>::quiet_NaN());
  splines_state.velocities.resize(dim, std::numeric_limits<double>::quiet_NaN());
  splines_state.accelerations.resize(dim, std::numeric_limits<double>::quiet_NaN());
  splines_state.effort.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_state.positions.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_state.velocities.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_state.accelerations.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_state.effort.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_input_state.positions.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_input_state.velocities.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_input_state.accelerations.resize(dim, std::numeric_limits<double>::quiet_NaN());
  ruckig_input_state.effort.resize(dim, std::numeric_limits<double>::quiet_NaN());

  if (!skip_splines)
  {
    auto generate_powers = [](int n, double x, double * powers)
    {
      powers[0] = 1.0;
      for (int i = 1; i <= n; ++i)
      {
        powers[i] = powers[i - 1] * x;
      }
    };

    if (duration_so_far.seconds() < 0.0)
    {
      duration_so_far = rclcpp::Duration::from_seconds(0.0);
      has_velocity = has_accel = false;
      RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Duration so far is negative");
    }
    if (duration_so_far.seconds() > duration_btwn_points.seconds())
    {
      RCLCPP_WARN(
        rclcpp::get_logger("trajectory"),
        "Duration so far is smaller then duration between points");
      duration_so_far = duration_btwn_points;
      has_velocity = has_accel = false;
    }

    output.effort = state_b.positions;

    double t[6];
    generate_powers(5, duration_so_far.seconds(), t);

    if (!has_velocity && !has_accel)
    {
      // do linear interpolation
      for (size_t i = 0; i < dim; ++i)
      {
        double start_pos = state_a.positions[i];
        double end_pos = state_b.positions[i];

        double coefficients[2] = {0.0, 0.0};
        coefficients[0] = start_pos;
        if (duration_btwn_points.seconds() != 0.0)
        {
          coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
        }

        output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
        output.velocities[i] = t[0] * coefficients[1];
      }
    }
    else if (has_velocity && !has_accel)
    {
      // do cubic interpolation
      double T[4];
      generate_powers(3, duration_btwn_points.seconds(), T);

      for (size_t i = 0; i < dim; ++i)
      {
        double start_pos = state_a.positions[i];
        double start_vel = state_a.velocities[i];
        double end_pos = state_b.positions[i];
        double end_vel = state_b.velocities[i];

        double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        if (duration_btwn_points.seconds() != 0.0)
        {
          coefficients[2] =
            (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
          coefficients[3] =
            (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
        }

        output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                              t[2] * coefficients[2] + t[3] * coefficients[3];
        output.velocities[i] =
          t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
        output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
      }
    }
    else if (has_velocity && has_accel)
    {
      // do quintic interpolation
      double T[6];
      generate_powers(5, duration_btwn_points.seconds(), T);

      for (size_t i = 0; i < dim; ++i)
      {
        double start_pos = state_a.positions[i];
        double start_vel = state_a.velocities[i];
        double start_acc = state_a.accelerations[i];
        double end_pos = state_b.positions[i];
        double end_vel = state_b.velocities[i];
        double end_acc = state_b.accelerations[i];

        double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        coefficients[2] = 0.5 * start_acc;
        if (duration_btwn_points.seconds() != 0.0)
        {
          coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] +
                             end_acc * T[2] - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) /
                            (2.0 * T[3]);
          coefficients[4] =
            (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] - 2.0 * end_acc * T[2] +
             16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
            (2.0 * T[4]);
          coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] +
                             end_acc * T[2] - 6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
                            (2.0 * T[5]);
        }

        output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                              t[2] * coefficients[2] + t[3] * coefficients[3] +
                              t[4] * coefficients[4] + t[5] * coefficients[5];
        output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
                               t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
                               t[4] * 5.0 * coefficients[5];
        output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
                                  t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
      }
    }
  }
  else
  {
    output.positions = state_b.positions;
    output.velocities = state_b.velocities;
    output.accelerations = state_b.accelerations;

    output.positions.resize(dim, 0.0);
    output.velocities.resize(dim, 0.0);
    output.accelerations.resize(dim, 0.0);
  }

  splines_state.positions = output.positions;
  splines_state.velocities = output.velocities;
  splines_state.accelerations = output.accelerations;
  splines_state.effort = output.effort;

  // Optionally apply velocity, acceleration, and jerk limits with Ruckig
  //   if ((duration_so_far.seconds() > 0 || skip_splines) && do_ruckig_smoothing)
  if (do_ruckig_smoothing)
  {
    // If Ruckig has run previously on this trajectory, use the output as input for the next cycle
    if (have_previous_ruckig_output_)
    {
      //       RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Applying previous Ruckig input");
      ruckig_input_state.effort[0] = 1;

      ruckig_input_.current_position = ruckig_output_.new_position;
      ruckig_input_.current_velocity = ruckig_output_.new_velocity;
      ruckig_input_.current_acceleration = ruckig_output_.new_acceleration;
    }
    // else, need to initialize to robot state
    else
    {
      ruckig_input_state.effort[0] = -1;

      ruckig_input_.current_position = state_a.positions;
      if (!state_a.velocities.empty())
      {
        ruckig_input_.current_velocity = state_a.velocities;
      }
      else
      {
        ruckig_input_.current_velocity = std::vector<double>(dim, 0.0);
      }
      if (!state_a.accelerations.empty())
      {
        ruckig_input_.current_acceleration = state_a.accelerations;
        ruckig_input_.current_acceleration = std::vector<double>(dim, 0.0);
      }
      else
      {
        ruckig_input_.current_acceleration = std::vector<double>(dim, 0.0);
      }
    }
    // Target state comes from the polynomial interpolation
    ruckig_input_.target_position = output.positions;

    // double max_vel_ratio = 1.0;
    // for (size_t i = 0; i < dim; ++i)
    // {
    //   if (std::fabs(output.velocities[i]) > joint_limits[i].max_velocity)
    //   {
    //     const double ratio = std::fabs(output.velocities[i] / joint_limits[i].max_velocity);
    //     if (ratio > max_vel_ratio)
    //     {
    //       max_vel_ratio = ratio;
    //     }
    //   }
    // }

    // for (size_t i = 0; i < dim; ++i)
    // {
    //   // Set the target velocities to follow the joint limits
    //   ruckig_input_.target_velocity[i] = output.velocities[i] / max_vel_ratio;
    //
    //   // Set the target accelerations to follow the joint limits
    //   ruckig_input_.target_acceleration[i] = std::clamp(
    //     output.accelerations[i], -1.0 * joint_limits[i].max_acceleration,
    //     joint_limits[i].max_acceleration);
    // }

    ruckig_input_state.positions = ruckig_input_.current_position;
    ruckig_input_state.velocities = ruckig_input_.current_velocity;
    ruckig_input_state.accelerations = ruckig_input_.current_acceleration;

    ruckig_state.positions = ruckig_input_.target_position;
    ruckig_state.velocities = ruckig_input_.target_velocity;
    ruckig_state.accelerations = ruckig_input_.target_acceleration;
    for (size_t i = 0; i < dim; ++i)
    {
      ruckig_state.effort[i] = ruckig_input_.target_acceleration[i] / period.seconds();
    }

    ruckig::Result result = smoother_->update(ruckig_input_, ruckig_output_);

    // If Ruckig was successful, update the output state
    // Else, just pass the output from the polynomial interpolation
    if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
    {
      ruckig_input_state.effort[1] = 1.1;
      have_previous_ruckig_output_ = true;
      output.positions = ruckig_output_.new_position;
      output.velocities = ruckig_output_.new_velocity;
      output.accelerations = ruckig_output_.new_acceleration;
    }
    else
    {
      ruckig_input_state.effort[1] = -1.1;
      if (result == ruckig::Result::ErrorInvalidInput)
      {
        RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Ruckig got invalid input");
        for (size_t i = 0; i < dim; ++i)
        {
          RCLCPP_WARN(
            rclcpp::get_logger("trajectory"),
            "Ruckig NOK input CURRENT pos: %.10f; vel: %.10f; acc: %.10f",
            ruckig_input_.current_position[i], ruckig_input_.current_velocity[i],
            ruckig_input_.current_acceleration[i]);
          RCLCPP_WARN(
            rclcpp::get_logger("trajectory"),
            "Ruckig NOK input TARGET pos: %.10f; vel: %.10f; acc: %.10f",
            ruckig_input_.target_position[i], ruckig_input_.target_velocity[i],
            ruckig_input_.target_acceleration[i]);
        }
      }
      RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Ruckig NOK!");
      return false;
    }
  }
  else
  {
    //     RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Skipping Ruckig");
  }

  return true;
}

void Trajectory::deduce_from_derivatives(
  trajectory_msgs::msg::JointTrajectoryPoint & first_state,
  trajectory_msgs::msg::JointTrajectoryPoint & second_state, const size_t dim, const double delta_t)
{
  if (second_state.positions.empty())
  {
    second_state.positions.resize(dim);
    if (first_state.velocities.empty())
    {
      first_state.velocities.resize(dim, 0.0);
    }
    if (second_state.velocities.empty())
    {
      second_state.velocities.resize(dim);
      if (first_state.accelerations.empty())
      {
        first_state.accelerations.resize(dim, 0.0);
      }
      for (size_t i = 0; i < dim; ++i)
      {
        second_state.velocities[i] =
          first_state.velocities[i] +
          (first_state.accelerations[i] + second_state.accelerations[i]) * 0.5 * delta_t;
      }
    }
    for (size_t i = 0; i < dim; ++i)
    {
      // second state velocity should be reached on the end of the segment, so use middle
      second_state.positions[i] =
        first_state.positions[i] +
        (first_state.velocities[i] + second_state.velocities[i]) * 0.5 * delta_t;
    }
  }
  else
  {
    for (size_t i = 0; i < dim; ++i)
    {
      if (std::isnan(second_state.positions[i]))
      {
        double first_state_velocity =
          first_state.velocities.empty() ? 0.0 : first_state.velocities[i];
        if (std::isnan(first_state_velocity))
        {
          first_state.velocities[i] = 0.0;
          first_state_velocity = 0.0;
        }
        double second_state_velocity =
          second_state.velocities.empty() ? 0.0 : second_state.velocities[i];
        if (std::isnan(second_state_velocity))
        {
          second_state.velocities[i] = 0.0;
          second_state_velocity = 0.0;
        }

        second_state.positions[i] =
          first_state.positions[i] + (first_state_velocity + second_state_velocity) * 0.5 * delta_t;
      }
    }
  }
}

TrajectoryPointConstIter Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

rclcpp::Time Trajectory::time_from_start() const { return trajectory_start_time_; }

bool Trajectory::has_trajectory_msg() const { return trajectory_msg_.get() != nullptr; }

bool Trajectory::has_nontrivial_msg() const
{
  return has_trajectory_msg() && trajectory_msg_->points.size() > 1;
}

}  // namespace joint_trajectory_controller
