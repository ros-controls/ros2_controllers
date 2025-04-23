// Copyright 2020 PAL Robotics S.L.
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

#include "forward_command_controller/forward_command_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace forward_command_controller
{
ForwardCommandController::ForwardCommandController() : ForwardControllersBase() {}

void ForwardCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ForwardCommandController::read_parameters()
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interface_types_.clear();
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
