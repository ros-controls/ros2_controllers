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
#include "hardware_interface/utils/time_utils.hpp"

#include "rclcpp/clock.hpp"

namespace ros_controllers
{

// TODO(karsten1987): Fix to rclcpp time when API stable.
using hardware_interface::utils::time_is_zero;
using hardware_interface::utils::time_less_than_equal;
using hardware_interface::utils::time_add;
using hardware_interface::utils::time_less_than;

Trajectory::Trajectory()
: trajectory_start_time_(0)
{}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(time_is_zero(joint_trajectory->header.stamp) ?
    rclcpp::Clock().now() :
    static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{}

void
Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = (time_is_zero(joint_trajectory->header.stamp) ?
    rclcpp::Clock().now() :
    static_cast<rclcpp::Time>(joint_trajectory->header.stamp));
}

TrajectoryPointConstIter
Trajectory::sample(const rclcpp::Time & sample_time)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  // skip if current time hasn't reached traj time of the first msg yet
  if (time_less_than(sample_time, trajectory_start_time_)) {
    return end();
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  for (auto point = begin(); point != end(); ++point) {
    auto start_time = time_add(trajectory_start_time_, point->time_from_start);
    if (time_less_than(sample_time, start_time)) {
      return point;
    }
  }

  return end();
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
Trajectory::is_empty() const
{
  return !trajectory_msg_;
}

}  // namespace ros_controllers
