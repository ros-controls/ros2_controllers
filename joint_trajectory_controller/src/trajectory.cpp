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

#include <memory>

#include "angles/angles.h"
#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("trajectory");
}  // anonymous namespace

namespace joint_trajectory_controller
{
Trajectory::Trajectory() : trajectory_start_time_(0), time_before_traj_msg_(0) {}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
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
  update(joint_trajectory);
}

void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
  const std::vector<bool> & joints_angle_wraparound)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;

  // If the current state doesn't contain velocities / accelerations, but the first trajectory
  // point does, initialize them to zero. Otherwise the segment going from the current state to the
  // first trajectory point will use another degree of spline interpolation than the rest of the
  // trajectory.
  if (current_point.velocities.empty() && !trajectory_msg_->points[0].velocities.empty())
  {
    state_before_traj_msg_.velocities.resize(trajectory_msg_->points[0].velocities.size(), 0.0);
  }
  if (current_point.accelerations.empty() && !trajectory_msg_->points[0].accelerations.empty())
  {
    state_before_traj_msg_.accelerations.resize(
      trajectory_msg_->points[0].accelerations.size(), 0.0);
  }

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

void Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;
  last_sample_idx_ = 0;
}

bool Trajectory::sample(
  const rclcpp::Time & sample_time,
  const interpolation_methods::InterpolationMethod interpolation_method,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state,
  TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr,
  const bool search_monotonically_increasing)
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

  output_state = trajectory_msgs::msg::JointTrajectoryPoint();
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
    }
    else
    {
      // it changes points only if position and velocity do not exist, but their derivatives
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        (first_point_timestamp - time_before_traj_msg_).seconds());

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time, output_state);
    }
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = last_sample_idx_; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // If interpolation is disabled, just forward the next waypoint
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      // Do interpolation
      else
      {
        // it changes points only if position and velocity do not exist, but their derivatives
        deduce_from_derivatives(
          point, next_point, state_before_traj_msg_.positions.size(), (t1 - t0).seconds());

        interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);
      }
      start_segment_itr = begin() + static_cast<TrajectoryPointConstIter::difference_type>(i);
      end_segment_itr = begin() + static_cast<TrajectoryPointConstIter::difference_type>(i + 1);
      output_state.time_from_start = next_point.time_from_start;
      if (search_monotonically_increasing)
      {
        last_sample_idx_ = i;
      }
      return true;
    }
  }

  // whole animation has played out
  start_segment_itr = --end();
  end_segment_itr = end();
  last_sample_idx_ = last_idx;
  output_state = (*start_segment_itr);
  // the trajectories in msg may have empty velocities/accel, so resize them
  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.effort.empty())
  {
    output_state.effort.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

void Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);
  output.effort.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers)
  {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
  bool has_effort = !state_a.effort.empty() && !state_b.effort.empty();
  if (duration_so_far.seconds() < 0.0)
  {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
    has_velocity = has_accel = false;
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds())
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);

  if (has_effort)
  {
    // do linear interpolation
    for (size_t i = 0; i < dim; ++i)
    {
      double start_effort = state_a.effort[i];
      double end_effort = state_b.effort[i];

      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = start_effort;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[1] = (end_effort - start_effort) / duration_btwn_points.seconds();
      }

      output.effort[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
    }
  }

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
        coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] -
                           2.0 * end_acc * T[2] + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
                          (2.0 * T[4]);
        coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
                           6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
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

void Trajectory::deduce_from_derivatives(
  trajectory_msgs::msg::JointTrajectoryPoint & first_state,
  trajectory_msgs::msg::JointTrajectoryPoint & second_state, const size_t dim, const double delta_t)
{
  if (first_state.effort.empty())
  {
    first_state.effort.assign(dim, 0.0);
  }
  if (second_state.effort.empty())
  {
    second_state.effort.assign(dim, 0.0);
  }
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

std::shared_ptr<trajectory_msgs::msg::JointTrajectory> decelerate_to_stop(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & state_current,
  const std::vector<double> & joint_accelerations,
  const std::vector<bool> & joints_angle_wraparound, const double sample_period)
{
  // validate inputs are the same size and the sample_period is not zero
  const size_t expected_size = joint_names.size();
  if (
    state_current.positions.size() != expected_size ||
    state_current.velocities.size() != expected_size ||
    joint_accelerations.size() != expected_size ||
    joints_angle_wraparound.size() != expected_size ||
    sample_period < std::numeric_limits<float>::epsilon())
  {
    RCLCPP_ERROR(
      kLogger,
      "Decelerate to stop calculations failed because input states did not match in size or the "
      "sample period was zero.");
    return nullptr;
  }
  auto num_joints = joint_names.size();
  // Precompute per-joint decel, stop time, and hold position
  std::vector<double> p0(num_joints), v0(num_joints), t_stop(num_joints), p_hold(num_joints),
    sgn(num_joints);
  double max_t_stop = 0.0;
  for (size_t i = 0; i < num_joints; ++i)
  {
    p0[i] = state_current.positions[i];
    v0[i] = state_current.velocities[i];

    sgn[i] = (v0[i] >= 0.0) ? 1.0 : -1.0;

    // Time to stop (constant decel)
    t_stop[i] = std::abs(v0[i]) / joint_accelerations[i];
    max_t_stop = std::max(max_t_stop, t_stop[i]);

    // Analytical stop distance and hold position
    const double stop_distance = (v0[i] * v0[i]) / (2.0 * joint_accelerations[i]);
    p_hold[i] = p0[i] + sgn[i] * stop_distance;

    RCLCPP_DEBUG(
      kLogger,
      "Joint [%s] decel [%.3f], stop dist [%.4f], initial vel [%.4f], initial pos [%.4f], hold pos "
      "[%.4f], time to stop [%.4f]",
      joint_names[i].c_str(), joint_accelerations[i], stop_distance, v0[i], p0[i], p_hold[i],
      t_stop[i]);
  }

  // Prepare output trajectory to stop each joint at max deceleration configured
  auto traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj->joint_names = joint_names;
  traj->points.clear();
  // Number of points at multiples of sample_period (include initial point at t=0)
  const size_t num_points = static_cast<size_t>(std::ceil(max_t_stop / sample_period)) + 1;
  traj->points.reserve(num_points);

  // Build traj points that ramp to zero velocity
  for (size_t k = 0; k < num_points; ++k)
  {
    const double t = static_cast<double>(k) * sample_period;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(num_joints);
    pt.velocities.resize(num_joints);
    pt.accelerations.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
      // if the joint still needs more time to stop and had an initial non-zero velocity
      if (t < t_stop[i] && std::abs(v0[i]) > std::numeric_limits<float>::epsilon())
      {
        // Constant deceleration
        // v(t) = v0 - sgn * a * t
        double v = v0[i] - sgn[i] * joint_accelerations[i] * t;
        // Guard against numerical crossing
        if ((v * sgn[i]) < 0.0) v = 0.0;
        // p(t) = p0 + v0 * t - 0.5 * sgn * a * t^2
        const double p = p0[i] + v0[i] * t - 0.5 * sgn[i] * joint_accelerations[i] * t * t;
        pt.positions[i] = p;
        pt.velocities[i] = v;
        pt.accelerations[i] = -sgn[i] * joint_accelerations[i];
      }
      else
      {
        // Joint is stopped, hold position and zero velocity/accel
        pt.positions[i] = p_hold[i];
        pt.velocities[i] = 0.0;
        pt.accelerations[i] = 0.0;
      }
    }
    // wrap the each stopping point around if the joint is continuous
    wraparound_joint(pt.positions, state_current.positions, joints_angle_wraparound);
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
    traj->points.emplace_back(std::move(pt));
  }

  RCLCPP_INFO(
    kLogger, "Created ramped stop trajectory with [%zu] points and max time to stop [%.3f] sec",
    traj->points.size(), max_t_stop);

  RCLCPP_INFO(
    kLogger, "Final joint positions [%.3f], [%.3f], [%.3f]", traj->points.back().positions[0],
    traj->points.back().positions[1], traj->points.back().positions[2]);

  return traj;
}

}  // namespace joint_trajectory_controller
