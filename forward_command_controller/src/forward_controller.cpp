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

#include "forward_command_controller/forward_controller.hpp"

#include "forward_command_controller/visibility_control.h"

namespace forward_command_controller
{
ForwardController::ForwardController()
: ForwardControllersBase(), controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration ForwardController::command_interface_configuration()
  const
{
  return get_command_interface_configuration();
}

controller_interface::InterfaceConfiguration ForwardController::state_interface_configuration()
  const
{
  return get_state_interface_configuration();
}

controller_interface::CallbackReturn ForwardController::on_init()
{
  return execute_init(get_node());
}

controller_interface::CallbackReturn ForwardController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return execute_configure(previous_state, command_interfaces_);
}

controller_interface::CallbackReturn ForwardController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return execute_activate(previous_state, command_interfaces_);
}

controller_interface::CallbackReturn ForwardController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return execute_deactivate(previous_state);
}

controller_interface::return_type ForwardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller
