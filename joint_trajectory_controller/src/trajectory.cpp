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

#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
namespace joint_trajectory_controller
{

Trajectory::Trajectory()
: trajectory_start_time_(0), time_before_traj_msg_(0)
{}

Trajectory::Trajectory(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
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
}

void
Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;
}

void
Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;
}

bool
Trajectory::sample(
  const rclcpp::Time & sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & expected_state,
  TrajectoryPointConstIter & start_segment_itr,
  TrajectoryPointConstIter & end_segment_itr)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  if (trajectory_msg_->points.empty()) {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  // first sampling of this trajectory
  if (!sampled_already_) {
    if (trajectory_start_time_.seconds() == 0.0) {
      trajectory_start_time_ = sample_time;
    }

    sampled_already_ = true;
  }

  auto linear_interpolation = [&](
    const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
    const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
    const rclcpp::Time & sample_time,
    trajectory_msgs::msg::JointTrajectoryPoint & output)
    {
      rclcpp::Duration duration_so_far = sample_time - time_a;
      rclcpp::Duration duration_btwn_points = time_b - time_a;
      double percent = duration_so_far.seconds() / duration_btwn_points.seconds();
      percent = percent > 1.0 ? 1.0 : percent;
      percent = percent < 0.0 ? 0.0 : percent;

      output.positions.resize(state_a.positions.size());
      for (auto i = 0ul; i < state_a.positions.size(); ++i) {
        output.positions[i] =
          state_a.positions[i] + percent * (state_b.positions[i] - state_a.positions[i]);
      }

      if (!state_a.velocities.empty() && !state_b.velocities.empty()) {
        output.velocities.resize(state_b.velocities.size());
        for (auto i = 0ul; i < state_b.velocities.size(); ++i) {
          output.velocities[i] =
            state_a.velocities[i] + percent * (state_b.velocities[i] - state_a.velocities[i]);
        }
      }

      if (!state_a.accelerations.empty() && !state_b.accelerations.empty()) {
        output.accelerations.resize(state_b.accelerations.size());
        for (auto i = 0ul; i < state_b.accelerations.size(); ++i) {
          output.accelerations[i] =
            state_a.accelerations[i] + percent *
            (state_b.accelerations[i] - state_a.accelerations[i]);
        }
      }
    };

  // current time hasn't reached traj time of the first msg yet
  const auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Duration offset = first_point_in_msg.time_from_start;
  const rclcpp::Time first_point_timestamp = trajectory_start_time_ + offset;
  if (sample_time < first_point_timestamp) {
    const rclcpp::Time t0 = time_before_traj_msg_;

    linear_interpolation(
      t0, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
      sample_time, expected_state);
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (auto i = 0ul; i < last_idx; ++i) {
    const auto & point = trajectory_msg_->points[i];
    const auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Duration t0_offset = point.time_from_start;
    const rclcpp::Duration t1_offset = next_point.time_from_start;
    const rclcpp::Time t0 = trajectory_start_time_ + t0_offset;
    const rclcpp::Time t1 = trajectory_start_time_ + t1_offset;

    if (sample_time >= t0 && sample_time < t1) {
      // TODO(ddengster): Find a way to add custom interpolation implementations.
      // Likely a lambda + parameters supplied from the controller would do
      // do simple linear interpolation for now
      // reference: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/joint_trajectory_controller/include/trajectory_interface/quintic_spline_segment.h#L84
      linear_interpolation(t0, point, t1, next_point, sample_time, expected_state);
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);
      return true;
    }
  }

  // whole animation has played out
  start_segment_itr = --end();
  end_segment_itr = end();
  expected_state = (*start_segment_itr);
  return true;
}

TrajectoryPointConstIter
Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter
Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

rclcpp::Time
Trajectory::time_from_start() const
{
  return trajectory_start_time_;
}

bool
Trajectory::has_trajectory_msg() const
{
  return !trajectory_msg_;
}

}  // namespace joint_trajectory_controller
