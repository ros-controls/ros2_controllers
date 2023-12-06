// Copyright (c) 2023 AIT Austrian Institute of Technology
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

#ifndef JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_
#define JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "joint_trajectory_controller_plugins/visibility_control.h"

namespace joint_trajectory_controller_plugins
{
class TrajectoryControllerBase
{
public:
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  TrajectoryControllerBase() = default;

  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual ~TrajectoryControllerBase() = default;

  /**
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool computeGains(const trajectory_msgs::msg::JointTrajectory trajectory) = 0;

  /**
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual void computeCommands(
    std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint current,
    const trajectory_msgs::msg::JointTrajectoryPoint error,
    const trajectory_msgs::msg::JointTrajectoryPoint desired, const rclcpp::Time & time,
    const rclcpp::Duration & period) = 0;

  /**
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual void reset() = 0;

protected:
  // the node handle for parameter handling
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace joint_trajectory_controller_plugins

#endif  // JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_
