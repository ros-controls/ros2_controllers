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

#ifndef ROS_CONTROLLERS__TRAJECTORY_HPP_
#define ROS_CONTROLLERS__TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "rclcpp/time.hpp"

#include "ros_controllers/visibility_control.h"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ros_controllers
{

using TrajectoryPointIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  ROS_CONTROLLERS_PUBLIC
  Trajectory();

  ROS_CONTROLLERS_PUBLIC
  explicit Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  ROS_CONTROLLERS_PUBLIC
  void
  update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// Find the next valid point from the containing trajectory msg.
  /**
  * Within each msg, points with time_from_start less or equal than current time will be skipped.
  * The first point with time_from_start greater than current time shall be a valid point,
  * Return end iterator if start time of desired trajectory msg is in the future
  * Else within each msg, valid point is the first point in the the msg with expected arrival time
  * in the future.
  * Arrival time is time_from_start of point + start time of msg
  * If an empty trajectory message is given, sample will return Trajectory::end()
  * If no valid point is found for the specified sample time, Trajectory::end() will be returned.
  */
  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  sample(const rclcpp::Time & sample_time);

  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  begin() const;

  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  end() const;

  ROS_CONTROLLERS_PUBLIC
  rclcpp::Time
  time_from_start() const;

  ROS_CONTROLLERS_PUBLIC
  bool
  is_empty() const;

private:
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;

  rclcpp::Time trajectory_start_time_;
};

}  // namespace ros_controllers

#endif  // ROS_CONTROLLERS__TRAJECTORY_HPP_
