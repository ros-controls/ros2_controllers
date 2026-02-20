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

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_trajectory_controller_plugins
{
class TrajectoryControllerBase
{
public:
  TrajectoryControllerBase() = default;

  virtual ~TrajectoryControllerBase() = default;

  /**
   * @brief get additional state interfaces required by this controller plugin
   */
  virtual std::vector<std::string> state_interface_configuration() const { return {}; }

  /**
   * @brief initialize the controller plugin.
   * @param node the node handle to use for parameter handling
   * @param map_cmd_to_joints a mapping from the joint names in the trajectory messages to the
   * command joints
   */
  bool initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<size_t> map_cmd_to_joints)
  {
    node_ = node;
    map_cmd_to_joints_ = map_cmd_to_joints;
    logger_ = this->set_logger();
    return on_initialize();
  };

  /**
   * @brief configure the controller plugin.
   */
  bool configure() { return on_configure(); }

  /**
   * @brief activate the controller plugin.
   */
  bool activate() { return on_activate(); }

  /**
   * @brief compute the control law for the given trajectory
   *
   * this method can take more time to compute the control law. Hence, it will block the execution
   * of the trajectory until it finishes
   *
   * this method is not virtual, any overrides won't be called by JTC. Instead, override
   * on_compute_control_law_non_rt for your implementation
   *
   * @return true if the gains were computed, false otherwise
   */
  bool compute_control_law_non_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory,
    const std::vector<double> & opt_state_interfaces_values)
  {
    rt_control_law_ready_ = false;
    auto ret = on_compute_control_law_non_rt(trajectory, opt_state_interfaces_values);
    rt_control_law_ready_ = true;
    return ret;
  }

  /**
   * @brief compute the control law for the given trajectory
   *
   * this method must finish quickly (within one controller-update rate)
   *
   * this method is not virtual, any overrides won't be called by JTC. Instead, override
   * on_compute_control_law_rt for your implementation
   *
   * @return true if the gains were computed, false otherwise
   */
  bool compute_control_law_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory,
    const std::vector<double> & opt_state_interfaces_values)
  {
    rt_control_law_ready_ = false;
    auto ret = on_compute_control_law_rt(trajectory, opt_state_interfaces_values);
    rt_control_law_ready_ = true;
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
  virtual bool update_gains_rt(void) = 0;

  /**
   * @brief compute the commands with the precalculated control law
   *
   * @param[out] tmp_command the output command
   * @param[out] scaling_fact scaling factor if ctrl is not following reference
   * @param[in] current the current state
   * @param[in] error the error between the current state and the desired state
   * @param[in] desired the desired state
   * @param[in] opt_state_interfaces_values optional state interface values, \ref
   * state_interface_configuration
   * @param[in] duration_since_start the duration since the start of the trajectory
   *            can be negative if the trajectory-start is in the future
   * @param[in] period the period since the last update
   */
  virtual void compute_commands(
    std::vector<double> & tmp_command, double & scaling_fact,
    const trajectory_msgs::msg::JointTrajectoryPoint current,
    const trajectory_msgs::msg::JointTrajectoryPoint error,
    const trajectory_msgs::msg::JointTrajectoryPoint desired,
    const std::vector<double> & opt_state_interfaces_values,
    const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period) = 0;

  /**
   * @brief reset internal states
   *
   * is called in on_deactivate() of JTC
   */
  virtual void reset() = 0;

  /**
   * @return true if the control law is ready (updated with the trajectory)
   */
  bool is_ready() { return rt_control_law_ready_; }

protected:
  // the node handle for parameter handling
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // map from joints in the message to command joints
  std::vector<size_t> map_cmd_to_joints_;

  /**
   * @brief Get the logger for this plugin
   */
  rclcpp::Logger get_logger() const { return logger_; }
  /**
   * @brief Get the logger for this plugin
   */
  virtual rclcpp::Logger set_logger() const = 0;

  /**
   * @brief compute the control law from the given trajectory (in the non-RT loop)
   * @return true if the gains were computed, false otherwise
   */
  virtual bool on_compute_control_law_non_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory,
    const std::vector<double> & opt_state_interfaces_values) = 0;

  /**
   * @brief compute the control law for a single point (in the RT loop)
   * @return true if the gains were computed, false otherwise
   */
  virtual bool on_compute_control_law_rt(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory,
    const std::vector<double> & opt_state_interfaces_values) = 0;

  /**
   * @brief initialize the controller plugin.
   *
   * declare parameters
   */
  virtual bool on_initialize(void) = 0;

  /**
   * @brief configure the controller plugin.
   *
   * parse read-only parameters, pre-allocate memory for the controller
   */
  virtual bool on_configure() = 0;

  /**
   * @brief activate the controller plugin.
   *
   * parse parameters
   */
  virtual bool on_activate() = 0;

private:
  // child logger for this plugin
  rclcpp::Logger logger_ = rclcpp::get_logger("joint_trajectory_controller_plugins");
  // Are we computing the control law or is it valid?
  std::atomic<bool> rt_control_law_ready_;
};

}  // namespace joint_trajectory_controller_plugins

#endif  // JOINT_TRAJECTORY_CONTROLLER_PLUGINS__TRAJECTORY_CONTROLLER_BASE_HPP_
