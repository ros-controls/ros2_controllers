// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "forward_command_controller/multi_interface_forward_command_controller.hpp"

#include <string>
#include <vector>

namespace forward_command_controller
{
MultiInterfaceForwardCommandController::MultiInterfaceForwardCommandController()
: ForwardControllersBase()
{
}

void MultiInterfaceForwardCommandController::declare_parameters()
{
  get_node()->declare_parameter<std::string>("joint", joint_name_);
  get_node()->declare_parameter<std::vector<std::string>>("interface_names", interface_names_);
}

controller_interface::CallbackReturn MultiInterfaceForwardCommandController::read_parameters()
{
  joint_name_ = get_node()->get_parameter("joint").as_string();
  interface_names_ = get_node()->get_parameter("interface_names").as_string_array();

  if (joint_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joint' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (interface_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interfaces' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & interface : interface_names_)
  {
    command_interface_types_.push_back(joint_name_ + "/" + interface);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::MultiInterfaceForwardCommandController,
  controller_interface::ControllerInterface)
