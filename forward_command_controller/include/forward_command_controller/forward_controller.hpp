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

#ifndef FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "forward_command_controller/forward_controllers_base.hpp"
#include "forward_command_controller/visibility_control.h"

namespace forward_command_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 */
class ForwardController : public ForwardControllersBase,
                          public controller_interface::ControllerInterface
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ForwardController();

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ~ForwardController() = default;

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

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLER_HPP_
