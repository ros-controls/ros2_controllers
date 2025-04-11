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

#include "controller_interface/controller_interface.hpp"
#include "effort_controllers/joint_group_effort_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"

namespace effort_controllers
{
JointGroupEffortController::JointGroupEffortController()
: forward_command_controller::ForwardCommandController()
{
  interface_name_ = hardware_interface::HW_IF_EFFORT;
}

controller_interface::CallbackReturn JointGroupEffortController::on_init()
{
  auto ret = forward_command_controller::ForwardCommandController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    // Explicitly set the interface parameter declared by the forward_command_controller
    // to match the value set in the JointGroupEffortController constructor.
    get_node()->set_parameter(
      rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointGroupEffortController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);
  // stop all joints
  for (auto & command_interface : command_interfaces_)
  {
    if (!command_interface.set_value(0.0))
    {
      RCLCPP_WARN(get_node()->get_logger(), "Error while setting command interface value to 0.0");
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  return ret;
}

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  effort_controllers::JointGroupEffortController, controller_interface::ControllerInterface)
