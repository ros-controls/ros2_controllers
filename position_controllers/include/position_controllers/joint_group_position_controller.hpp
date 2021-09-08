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

#ifndef POSITION_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_

#include <string>

#include "forward_command_controller/forward_command_controller.hpp"
#include "position_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace position_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Forward command controller for a set of position controlled joints (linear or angular).
 *
 * This class forwards the commanded positions down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The position commands to apply.
 */
class JointGroupPositionController : public forward_command_controller::ForwardCommandController
{
public:
  POSITION_CONTROLLERS_PUBLIC
  JointGroupPositionController();

  POSITION_CONTROLLERS_PUBLIC CallbackReturn on_init() override;
};

}  // namespace position_controllers

#endif  // POSITION_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_HPP_
