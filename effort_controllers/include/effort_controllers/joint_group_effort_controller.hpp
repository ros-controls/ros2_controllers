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

#ifndef EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
#define EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_

#include <string>

#include "effort_controllers/visibility_control.h"
#include "forward_command_controller/forward_command_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace effort_controllers
{
/**
 * \brief Forward command controller for a set of effort controlled joints (linear or angular).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64MultiArray) : The effort commands to apply.
 */
class JointGroupEffortController : public forward_command_controller::ForwardCommandController
{
public:
  EFFORT_CONTROLLERS_PUBLIC
  JointGroupEffortController();

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
};

}  // namespace effort_controllers

#endif  // EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
