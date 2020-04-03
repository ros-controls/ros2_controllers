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

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "joint_trajectory_controller/visibility_control.h"

#include "rclcpp/time.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace joint_trajectory_controller
{

using TrajectoryPointIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  Trajectory();

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  explicit Trajectory(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  explicit Trajectory(
    const rclcpp::Time & current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// Set the point before the trajectory message is replaced/appended
  /// Example: if we receive a new trajectory message and it's first point is 0.5 seconds
  /// from the current one, we call this function to log the current state, then
  /// append/replace the current trajectory
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void
  set_point_before_trajectory_msg(
    const rclcpp::Time & current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_point);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void
  update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// Find the segment (made up of 2 points) and its expected state from the
  /// containing trajectory.
  /**
  * Specific case returns for start_segment_itr and end_segment_itr:
  * - Sampling before the trajectory start:
  *   start_segment_itr = begin(), end_segment_itr = begin()
  * - Sampling exactly on a point of the trajectory:
  *    start_segment_itr = iterator where point is, end_segment_itr = iterator after start_segment_itr
  * - Sampling between points:
  *    start_segment_itr = iterator before the sampled point, end_segment_itr = iterator after start_segment_itr
  * - Sampling after entire trajectory:
  *    start_segment_itr = --end(), end_segment_itr = end()
  * - Sampling empty msg:
  *    return false
  */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool
  sample(
    const rclcpp::Time & sample_time,
    trajectory_msgs::msg::JointTrajectoryPoint & expected_state,
    TrajectoryPointConstIter & start_segment_itr,
    TrajectoryPointConstIter & end_segment_itr);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter
  begin() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter
  end() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp::Time
  time_from_start() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool
  has_trajectory_msg() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
  get_trajectory_msg() const {return trajectory_msg_;}

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp::Time get_trajectory_start_time() const {return trajectory_start_time_;}

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool is_sampled_already() const {return sampled_already_;}

private:
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  rclcpp::Time trajectory_start_time_;

  rclcpp::Time time_before_traj_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint state_before_traj_msg_;

  bool sampled_already_ = false;
};

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
