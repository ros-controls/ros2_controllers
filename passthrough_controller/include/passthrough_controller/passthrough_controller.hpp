// Copyright (c) 2023, PAL Robotics
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

#ifndef PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_
#define PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_

#include <controller_interface/chainable_controller_interface.hpp>

#include "passthrough_controller/visibility_control.h"

namespace passthrough_controller
{
class PassthroughController : public controller_interface::ChainableControllerInterface
{
public:
  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  PassthroughController(){}

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    return command_interfaces_config;
  }

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    return state_interfaces_config;
  }

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override
  {
    return controller_interface::return_type::OK;
  }

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    return std::vector<hardware_interface::CommandInterface>();
  }

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  bool on_set_chained_mode(bool /* chained_mode */) override
  {
    return true;
  }

  PASSTHROUGH_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override
  {
    return controller_interface::return_type::OK;
  }
};

}  // namespace passthrough_controller

#endif  // PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_
