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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.h"
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
   * @brief initialize the controller plugin.
   * declare parameters
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief configure the controller plugin.
   * parse read-only parameters, pre-allocate memory for the controller
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool configure() = 0;

  /**
   * @brief activate the controller plugin.
   * parse parameters
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool activate() = 0;

  /**
   * @brief compute the control law for the given trajectory
   *
   * this method can take more time to compute the control law. Hence, it will block the execution
   * of the trajectory until it finishes
   *
   * this method is not virtual, any overrides won't be called by JTC. Instead, override
   * computeControlLaw for your implementation
   *
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  bool computeControlLawNonRT(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
  {
    rt_control_law_ready_.writeFromNonRT(false);
    auto ret = computeControlLaw(trajectory);
    rt_control_law_ready_.writeFromNonRT(true);
    return ret;
  }

  /**
   * @brief set the time when the current trajectory started
   *
   * (same logic as trajectory class)
   */
  void start(const rclcpp::Time time, const rclcpp::Time trajectory_start_time)
  {
    if (trajectory_start_time.seconds() == 0.0)
    {
      trajectory_start_time_ = time;
    }
    else
    {
      trajectory_start_time_ = trajectory_start_time;
    }
    start_trajectory_ = true;
  }

  /**
   * @brief update the gains from a RT thread
   *
   * this method must finish quickly (within one controller-update rate)
   *
   * @return true if the gains were updated, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool updateGainsRT(void) = 0;

  /**
   * @brief compute the commands with the calculated gains
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual void computeCommands(
    std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint current,
    const trajectory_msgs::msg::JointTrajectoryPoint error,
    const trajectory_msgs::msg::JointTrajectoryPoint desired, const rclcpp::Time & time,
    const rclcpp::Duration & period) = 0;

  /**
   * @brief reset internal states
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual void reset() = 0;

  /**
   * @return true if the control law is ready (updated with the trajectory)
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  bool is_ready() { return rt_control_law_ready_.readFromRT(); }

protected:
  // the node handle for parameter handling
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // Are we computing the control law or is it valid?
  realtime_tools::RealtimeBuffer<bool> rt_control_law_ready_;
  // time when the current trajectory started, can be used to interpolate time-varying gains
  rclcpp::Time trajectory_start_time_;
  // use this variable to activate new gains from the non-RT thread
  bool start_trajectory_ = false;

  /**
   * @brief compute the control law from the given trajectory
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool computeControlLaw(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory) = 0;
};

}  // namespace joint_trajectory_controller_plugins

#endif  // JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_
