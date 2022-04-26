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

#include "forward_command_controller/chainable_forward_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ChainableForwardController::ChainableForwardController()
: ForwardControllersBase(), controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn ChainableForwardController::on_init()
{
  return execute_init(get_node());
}

controller_interface::CallbackReturn ChainableForwardController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = execute_configure(previous_state, command_interfaces_);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // The names should be in the same order as for command interfaces for easier matching
  reference_interface_names_ = command_interface_names_;
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(
    reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ChainableForwardController::command_interface_configuration() const
{
  return get_command_interface_configuration();
}

controller_interface::InterfaceConfiguration
ChainableForwardController::state_interface_configuration() const
{
  return get_state_interface_configuration();
}

std::vector<hardware_interface::CommandInterface>
ChainableForwardController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool ChainableForwardController::on_set_chained_mode(bool chained_mode)
{
  // we can set chained mode in any situation
  (void)chained_mode;
  return true;
}

controller_interface::CallbackReturn ChainableForwardController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = execute_activate(previous_state, command_interfaces_);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainableForwardController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return execute_deactivate(previous_state);
}

controller_interface::return_type ChainableForwardController::update_reference_from_subscribers()
{
  auto joint_commands = rt_command_ptr_.readFromRT();
  // message is valid
  if (!(!joint_commands || !(*joint_commands)))
  {
    reference_interfaces_ = (*joint_commands)->data;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ChainableForwardController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan(reference_interfaces_[i]))
    {
      command_interfaces_[i].set_value(reference_interfaces_[i]);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller
