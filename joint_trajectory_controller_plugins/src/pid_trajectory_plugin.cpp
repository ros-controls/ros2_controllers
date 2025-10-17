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

bool PidTrajectoryPlugin::on_initialize()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(node_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  return true;
}

bool PidTrajectoryPlugin::on_configure()
{
  try
  {
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return false;
  }

  // parse read-only params
  num_cmd_joints_ = params_.command_joints.size();
  if (num_cmd_joints_ == 0)
  {
    RCLCPP_ERROR(get_logger(), "No command joints specified.");
    return false;
  }
  if (num_cmd_joints_ != map_cmd_to_joints_.size())
  {
    RCLCPP_ERROR(get_logger(), "map_cmd_to_joints has to be of size num_cmd_joints.");
    return false;
  }
  pids_.resize(num_cmd_joints_);  // memory for the shared pointers, will be nullptr
  // create the objects with default values
  for (size_t i = 0; i < num_cmd_joints_; ++i)
  {
    pids_[i] = std::make_shared<control_toolbox::Pid>();
  }
  ff_velocity_scale_.resize(num_cmd_joints_, 0.0);

  return true;
}

bool PidTrajectoryPlugin::on_activate()
{
  params_ = param_listener_->get_params();
  parse_gains();
  return true;
}

bool PidTrajectoryPlugin::update_gains_rt()
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    parse_gains();
  }

  return true;
}

void PidTrajectoryPlugin::parse_gains()
{
  for (size_t i = 0; i < num_cmd_joints_; ++i)
  {
    RCLCPP_DEBUG(
      get_logger(), "params_.command_joints %lu : %s", i, params_.command_joints[i].c_str());

    const auto & gains = params_.gains.command_joints_map.at(params_.command_joints[i]);
    control_toolbox::AntiWindupStrategy antiwindup_strat;
    antiwindup_strat.set_type(gains.antiwindup_strategy);
    antiwindup_strat.i_max = gains.i_clamp_max;
    antiwindup_strat.i_min = gains.i_clamp_min;
    antiwindup_strat.error_deadband = gains.error_deadband;
    antiwindup_strat.tracking_time_constant = gains.tracking_time_constant;
    pids_[i]->set_gains(
      gains.p, gains.i, gains.d, gains.u_clamp_max, gains.u_clamp_min, antiwindup_strat);
    ff_velocity_scale_[i] = gains.ff_velocity_scale;

    RCLCPP_DEBUG(get_logger(), "gains.p: %f", gains.p);
    RCLCPP_DEBUG(get_logger(), "ff_velocity_scale_: %f", ff_velocity_scale_[i]);
  }

  RCLCPP_INFO(
    get_logger(), "Loaded PID gains from ROS parameters for %lu joint(s).", num_cmd_joints_);
}

void PidTrajectoryPlugin::compute_commands(
  std::vector<double> & tmp_command, double & /*scaling_fact*/,
  const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired,
  const std::vector<double> & /* opt_state_interfaces_values*/,
  const rclcpp::Duration & /*duration_since_start*/, const rclcpp::Duration & period)
{
  // if effort field is present, otherwise it would have been rejected
  auto has_effort_command_interface = !desired.effort.empty();
  // Update PIDs
  for (auto i = 0ul; i < num_cmd_joints_; ++i)
  {
    const auto map_cmd_to_joint = map_cmd_to_joints_[i];
    tmp_command[map_cmd_to_joint] =
      (desired.velocities[map_cmd_to_joint] * ff_velocity_scale_[i]) +
      (has_effort_command_interface ? desired.effort[i] : 0.0) +
      pids_[i]->compute_command(
        error.positions[map_cmd_to_joint], error.velocities[map_cmd_to_joint], period);
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
