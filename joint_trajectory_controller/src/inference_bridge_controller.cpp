// Copyright (c) 2026 ros2_control Development Team
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

#include "joint_trajectory_controller/inference_bridge_controller.hpp"

#include <exception>
#include <memory>

#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"

namespace inference_bridge_controller
{

InferenceBridgeController::InferenceBridgeController()
: joint_trajectory_controller::JointTrajectoryController()
{
}

controller_interface::CallbackReturn InferenceBridgeController::on_init()
{
  const auto ret = JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    bridge_param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception initializing bridge parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InferenceBridgeController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  bridge_params_ = bridge_param_listener_->get_params();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "inference_bridge_controller configured: policy_frequency=%.2f Hz.",
    bridge_params_.policy_frequency);

  return controller_interface::CallbackReturn::SUCCESS;
}

void InferenceBridgeController::topic_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  // TODO: add staleness / max-age check for stamp=0 chunks

  if (is_positions_only(*msg))
  {
    synthesize_timing(*msg);
    joint_trajectory_controller::fill_cubic_spline_velocities(*msg);
  }

  // TODO: temporal ensembling (blend overlapping chunks instead of newest-wins)

  JointTrajectoryController::topic_callback(msg);
}

bool InferenceBridgeController::is_positions_only(
  const trajectory_msgs::msg::JointTrajectory & traj) const
{
  if (traj.points.empty())
  {
    return false;
  }
  for (const auto & point : traj.points)
  {
    if (point.positions.empty())
    {
      return false;
    }
    if (!point.velocities.empty())
    {
      return false;  // derivatives already provided -> pass through to JTC unchanged
    }
  }
  return true;
}

void InferenceBridgeController::synthesize_timing(
  trajectory_msgs::msg::JointTrajectory & traj) const
{
  // Respect timing the policy already set; only synthesize when it is absent.
  for (size_t i = 1; i < traj.points.size(); ++i)
  {
    const auto & t = traj.points[i].time_from_start;
    if (t.sec != 0 || t.nanosec != 0u)
    {
      return;
    }
  }

  const double dt = 1.0 / bridge_params_.policy_frequency;
  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    traj.points[i].time_from_start =
      rclcpp::Duration::from_seconds(static_cast<double>(i) * dt);
  }
}

}  // namespace inference_bridge_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inference_bridge_controller::InferenceBridgeController, controller_interface::ControllerInterface)
