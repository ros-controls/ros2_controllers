// Copyright 2024 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "forward_state_controller/forward_state_controller.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/logging.hpp"

namespace forward_state_controller
{
ForwardStateController::ForwardStateController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ForwardStateController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardStateController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interfaces' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  state_interface_names_.clear();
  command_interface_names_.clear();
  state_to_command_map_.clear();

  for (const auto & state_iface_name : params_.state_interfaces)
  {
    auto it = params_.forward_state.state_interfaces_map.find(state_iface_name);
    if (it == params_.forward_state.state_interfaces_map.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "No 'forward_state.%s' mapping found in parameters", state_iface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto & to_command = it->second.to_command;
    if (to_command.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "'forward_state.%s.to_command' is empty", state_iface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const std::size_t state_idx = state_interface_names_.size();
    state_interface_names_.push_back(state_iface_name);

    std::vector<std::size_t> cmd_indices;
    for (const auto & cmd_iface_name : to_command)
    {
      // deduplicate command_interface_names_
      auto cmd_it = std::find(
        command_interface_names_.begin(), command_interface_names_.end(), cmd_iface_name);
      std::size_t cmd_idx;
      if (cmd_it == command_interface_names_.end())
      {
        cmd_idx = command_interface_names_.size();
        command_interface_names_.push_back(cmd_iface_name);
      }
      else
      {
        cmd_idx = static_cast<std::size_t>(std::distance(command_interface_names_.begin(), cmd_it));
      }
      cmd_indices.push_back(cmd_idx);
    }
    state_to_command_map_[state_idx] = cmd_indices;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardStateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForwardStateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;

  return state_interfaces_config;
}

controller_interface::CallbackReturn ForwardStateController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardStateController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardStateController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & [state_idx, cmd_indices] : state_to_command_map_)
  {
    const auto state_value_opt = state_interfaces_[state_idx].get_optional<double>();
    if (!state_value_opt.has_value())
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Unable to get value from state interface '%s'",
        state_interfaces_[state_idx].get_name().c_str());
      continue;
    }

    const double value = state_value_opt.value();
    for (const auto & cmd_idx : cmd_indices)
    {
      command_interfaces_[cmd_idx].set_value(value)
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_state_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_state_controller::ForwardStateController, controller_interface::ControllerInterface)
