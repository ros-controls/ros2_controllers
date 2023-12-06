// Copyright 2023 AIT Austrian Institute of Technology
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

#include "joint_trajectory_controller_plugins/pid_trajectory_plugin.hpp"

namespace joint_trajectory_controller_plugins
{

bool PidTrajectoryPlugin::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  node_ = node;

  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(node_);
    params_ = param_listener_->get_params();
    dof_ = params_.joints.size();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "[PidTrajectoryPlugin] Initialized with %lu joints.", dof_);
  return true;
}

bool PidTrajectoryPlugin::computeGains(const trajectory_msgs::msg::JointTrajectory /*trajectory*/)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_DEBUG(node_->get_logger(), "[PidTrajectoryPlugin] Updated parameters");
  }

  pids_.resize(dof_);
  ff_velocity_scale_.resize(dof_);

  // Init PID gains from ROS parameters
  for (size_t i = 0; i < dof_; ++i)
  {
    const auto & gains = params_.gains.joints_map.at(params_.joints[i]);
    pids_[i] = std::make_shared<control_toolbox::Pid>(
      gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);

    ff_velocity_scale_[i] = gains.ff_velocity_scale;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PidTrajectoryPlugin] Loaded PID gains from ROS parameters for %lu joints.", dof_);
  return true;
}

void PidTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired, const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Update PIDs
  for (auto i = 0ul; i < dof_; ++i)
  {
    tmp_command[i] = (desired.velocities[i] * ff_velocity_scale_[i]) +
                     pids_[i]->computeCommand(
                       error.positions[i], error.velocities[i], (uint64_t)period.nanoseconds());
  }
}

void PidTrajectoryPlugin::reset()
{
  for (const auto & pid : pids_)
  {
    if (pid)
    {
      pid->reset();
    }
  }
}

}  // namespace joint_trajectory_controller_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller_plugins::PidTrajectoryPlugin,
  joint_trajectory_controller_plugins::TrajectoryControllerBase)
