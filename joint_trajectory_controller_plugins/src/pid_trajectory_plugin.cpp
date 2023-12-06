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

bool PidTrajectoryPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<std::string> command_joint_names)
{
  node_ = node;
  command_joint_names_ = command_joint_names;

  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(node_);
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  // parse read-only params
  num_cmd_joints_ = command_joint_names_.size();
  if (num_cmd_joints_ == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[PidTrajectoryPlugin] No command joints specified.");
    return false;
  }
  pids_.resize(num_cmd_joints_);
  ff_velocity_scale_.resize(num_cmd_joints_);

  return true;
}

bool PidTrajectoryPlugin::updateGainsRT()
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    updateGains();
  }

  return true;
}

bool PidTrajectoryPlugin::computeGainsNonRT(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & /*trajectory*/)
{
  params_ = param_listener_->get_params();
  updateGains();
  return true;
};

void PidTrajectoryPlugin::updateGains()
{
  for (size_t i = 0; i < num_cmd_joints_; ++i)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "[PidTrajectoryPlugin] command_joint_names_ %lu : %s", i,
      command_joint_names_[i].c_str());

    const auto & gains = params_.gains.command_joints_map.at(command_joint_names_[i]);
    pids_[i] = std::make_shared<control_toolbox::Pid>(
      gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);
    ff_velocity_scale_[i] = gains.ff_velocity_scale;

    RCLCPP_DEBUG(node_->get_logger(), "[PidTrajectoryPlugin] gains.p: %f", gains.p);
    RCLCPP_DEBUG(
      node_->get_logger(), "[PidTrajectoryPlugin] ff_velocity_scale_: %f", ff_velocity_scale_[i]);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PidTrajectoryPlugin] Loaded PID gains from ROS parameters for %lu joint(s).",
    num_cmd_joints_);
}

void PidTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired, const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Update PIDs
  for (auto i = 0ul; i < num_cmd_joints_; ++i)
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
