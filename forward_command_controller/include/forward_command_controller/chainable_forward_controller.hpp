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

#ifndef FORWARD_COMMAND_CONTROLLER__CHAINABLE_FORWARD_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__CHAINABLE_FORWARD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "forward_command_controller/forward_controllers_base.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace forward_command_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 *
 * This class forwards the command signal down to a set of joints or interfaces.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class ChainableForwardController : public ForwardControllersBase,
                                   public controller_interface::ChainableControllerInterface
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ChainableForwardController();

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ~ChainableForwardController() = default;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<std::string> reference_interface_names_;
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__CHAINABLE_FORWARD_CONTROLLER_HPP_
