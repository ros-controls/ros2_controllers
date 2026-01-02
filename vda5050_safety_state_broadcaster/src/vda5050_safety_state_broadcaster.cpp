// Copyright (c) 2025, b-robotized
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include <vda5050_safety_state_broadcaster/vda5050_safety_state_broadcaster.hpp>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace vda5050_safety_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
const size_t MAX_LENGTH = 64;  // maximum length of strings to reserve

Vda5050SafetyStateBroadcaster::Vda5050SafetyStateBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn Vda5050SafetyStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<vda5050_safety_state_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Vda5050SafetyStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  try
  {
    vda5050_safety_state_publisher_ =
      get_node()->create_publisher<control_msgs::msg::VDA5050SafetyState>(
        "~/vda5050_safety_state", rclcpp::SystemDefaultsQoS());

    realtime_vda5050_safety_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::VDA5050SafetyState>>(
        vda5050_safety_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!realtime_vda5050_safety_state_publisher_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Realtime publisher not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }
  safety_state_msg.e_stop.reserve(MAX_LENGTH);

  // Initialize the indices for different interface types.
  itfs_ids_ = {};
  itfs_ids_.manual_start = static_cast<int>(params_.fieldViolation_interfaces.size());
  itfs_ids_.remote_start =
    itfs_ids_.manual_start + static_cast<int>(params_.eStop_manual_interfaces.size());
  itfs_ids_.autoack_start =
    itfs_ids_.remote_start + static_cast<int>(params_.eStop_remote_interfaces.size());
  itfs_ids_.total_interfaces =
    itfs_ids_.autoack_start + static_cast<int>(params_.eStop_autoack_interfaces.size());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Vda5050SafetyStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
Vda5050SafetyStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(itfs_ids_.total_interfaces);
  for (auto const & fieldViolation_interface : params_.fieldViolation_interfaces)
  {
    state_interfaces_config.names.push_back(fieldViolation_interface);
  }
  for (auto const & eStop_manual_interface : params_.eStop_manual_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_manual_interface);
  }
  for (auto const & eStop_remote_interface : params_.eStop_remote_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_remote_interface);
  }
  for (auto const & eStop_autoack_interface : params_.eStop_autoack_interfaces)
  {
    state_interfaces_config.names.push_back(eStop_autoack_interface);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn Vda5050SafetyStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces found to publish.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (static_cast<size_t>(itfs_ids_.total_interfaces) != state_interfaces_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Number of configured interfaces (%d) does not match number of provided state interfaces "
      "(%zu).",
      itfs_ids_.total_interfaces, state_interfaces_.size());
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!realtime_vda5050_safety_state_publisher_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Realtime publisher not initialized");
    return controller_interface::CallbackReturn::FAILURE;
  }

  safety_state_msg.e_stop = control_msgs::msg::VDA5050SafetyState::NONE;
  safety_state_msg.field_violation = false;
  realtime_vda5050_safety_state_publisher_->try_publish(safety_state_msg);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Vda5050SafetyStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Vda5050SafetyStateBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  fieldViolation_value = false;
  for (int itf_idx = 0; itf_idx < itfs_ids_.manual_start; ++itf_idx)
  {
    if (safe_double_to_bool(
          state_interfaces_[itf_idx].get_optional().value_or(kUninitializedValue)))
    {
      fieldViolation_value = true;
      break;
    }
  }

  estop_msg = determineEstopState();

  if (realtime_vda5050_safety_state_publisher_)
  {
    safety_state_msg.field_violation = fieldViolation_value;
    safety_state_msg.e_stop = estop_msg;
    realtime_vda5050_safety_state_publisher_->try_publish(safety_state_msg);
  }

  return controller_interface::return_type::OK;
}

control_msgs::msg::VDA5050SafetyState::_e_stop_type
Vda5050SafetyStateBroadcaster::determineEstopState()
{
  // Scan all e-stop interfaces and return the type of the first active one
  for (int itf_idx = itfs_ids_.manual_start; itf_idx < itfs_ids_.total_interfaces; ++itf_idx)
  {
    if (safe_double_to_bool(
          state_interfaces_[itf_idx].get_optional().value_or(kUninitializedValue)))
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "E-stop triggered by interface %s",
        state_interfaces_[itf_idx].get_name().c_str());
      if (itf_idx < itfs_ids_.remote_start)
      {
        return control_msgs::msg::VDA5050SafetyState::MANUAL;
      }
      else if (itf_idx < itfs_ids_.autoack_start)
      {
        return control_msgs::msg::VDA5050SafetyState::REMOTE;
      }
      else
      {
        return control_msgs::msg::VDA5050SafetyState::AUTO_ACK;
      }
    }
  }

  return control_msgs::msg::VDA5050SafetyState::NONE;
}

}  // namespace vda5050_safety_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vda5050_safety_state_broadcaster::Vda5050SafetyStateBroadcaster,
  controller_interface::ControllerInterface)
