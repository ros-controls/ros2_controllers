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

#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "velocity_controllers/joint_group_velocity_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace velocity_controllers
{
using CallbackReturn = JointGroupVelocityController::CallbackReturn;

JointGroupVelocityController::JointGroupVelocityController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "joint velocity controller";
  interface_name_ = hardware_interface::HW_IF_VELOCITY;
}

controller_interface::return_type
JointGroupVelocityController::init(
  const std::string & controller_name)
{
  auto ret = ForwardCommandController::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    // Explicitly set the interface parameter declared by the forward_command_controller
    // to match the value set in the JointGroupVelocityController constructor.
    get_node()->set_parameter(rclcpp::Parameter("interface_name", hardware_interface::HW_IF_VELOCITY));

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointGroupVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  // stop all joints
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return ret;
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::JointGroupVelocityController, controller_interface::ControllerInterface)
