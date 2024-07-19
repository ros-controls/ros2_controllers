// Copyright 2014, SRI International
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

/// \author Sachin Chitta

// Project
#include <gripper_controllers/gripper_action_controller.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
namespace position_controllers
{
/**
 * \brief Gripper action controller that sends
 * commands to a \b position interface.
 */
using GripperActionController =
  gripper_action_controller::GripperActionController<hardware_interface::HW_IF_POSITION>;
}  // namespace position_controllers

namespace effort_controllers
{
/**
 * \brief Gripper action controller that sends
 * commands to a \b effort interface.
 */
using GripperActionController =
  gripper_action_controller::GripperActionController<hardware_interface::HW_IF_EFFORT>;
}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  position_controllers::GripperActionController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(
  effort_controllers::GripperActionController, controller_interface::ControllerInterface)
