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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ForwardCommandController::ForwardCommandController() : ForwardControllersBase() {}

void ForwardCommandController::declare_parameters()
{
  get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
  get_node()->declare_parameter<std::string>("interface_name", "");
}

controller_interface::CallbackReturn ForwardCommandController::read_parameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling configure function.
  if (interface_name_.empty())
  {
    interface_name_ = get_node()->get_parameter("interface_name").as_string();
  }

  if (interface_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : joint_names_)
  {
    command_interface_types_.push_back(joint + "/" + interface_name_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
