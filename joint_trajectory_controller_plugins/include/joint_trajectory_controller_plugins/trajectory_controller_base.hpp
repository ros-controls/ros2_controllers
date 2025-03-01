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
#include "realtime_tools/realtime_buffer.hpp"
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
   * @param node the node handle to use for parameter handling
   * @param map_cmd_to_joints a mapping from the joint names in the trajectory messages to the
   * command joints
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<size_t> map_cmd_to_joints) = 0;

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
   * compute_control_law_non_rt_impl for your implementation
   *
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  bool compute_control_law_non_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
  {
    rt_control_law_ready_.writeFromNonRT(false);
    auto ret = compute_control_law_non_rt_impl(trajectory);
    rt_control_law_ready_.writeFromNonRT(true);
    return ret;
  }

  /**
   * @brief compute the control law for the given trajectory
   *
   * this method must finish quickly (within one controller-update rate)
   *
   * this method is not virtual, any overrides won't be called by JTC. Instead, override
   * compute_control_law_rt_impl for your implementation
   *
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  bool compute_control_law_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
  {
    // TODO(christophfroehlich): Need a lock-free write here
    rt_control_law_ready_.writeFromNonRT(false);
    auto ret = compute_control_law_rt_impl(trajectory);
    rt_control_law_ready_.writeFromNonRT(true);
    return ret;
  }

  /**
   * @brief called when the current trajectory started in update loop
   *
   * use this to implement switching of real-time buffers for updating the control law
   */
  virtual void start(void) = 0;

  /**
   * @brief update the gains from a RT thread
   *
   * this method must finish quickly (within one controller-update rate)
   *
   * @return true if the gains were updated, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool update_gains_rt(void) = 0;

  /**
   * @brief compute the commands with the precalculated control law
   *
   * @param[out] tmp_command the output command
   * @param[in] current the current state
   * @param[in] error the error between the current state and the desired state
   * @param[in] desired the desired state
   * @param[in] duration_since_start the duration since the start of the trajectory
   *            can be negative if the trajectory-start is in the future
   * @param[in] period the period since the last update
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual void compute_commands(
    std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint current,
    const trajectory_msgs::msg::JointTrajectoryPoint error,
    const trajectory_msgs::msg::JointTrajectoryPoint desired,
    const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period) = 0;

  /**
   * @brief reset internal states
   *
   * is called in on_deactivate() of JTC
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

  /**
   * @brief compute the control law from the given trajectory (in the non-RT loop)
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool compute_control_law_non_rt_impl(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory) = 0;

  /**
   * @brief compute the control law for a single point (in the RT loop)
   * @return true if the gains were computed, false otherwise
   */
  JOINT_TRAJECTORY_CONTROLLER_PLUGINS_PUBLIC
  virtual bool compute_control_law_rt_impl(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory) = 0;
};

}  // namespace joint_trajectory_controller_plugins

#endif  // JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_
