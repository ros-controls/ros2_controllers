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

#ifndef FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace forward_command_controller
{

class ForwardCommandController : public controller_interface::ControllerInterface
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ForwardCommandController();

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type
  update() override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_
