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

#include <inttypes.h>
#include <string>

#include "angles/angles.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "effort_controllers/joint_group_position_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace effort_controllers
{
using CallbackReturn = JointGroupPositionController::CallbackReturn;

JointGroupPositionController::JointGroupPositionController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "joint effort controller";
  interface_name_ = hardware_interface::HW_IF_EFFORT;
}

controller_interface::return_type
JointGroupPositionController::init(
  const std::string & controller_name)
{
  auto ret = ForwardCommandController::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    // undeclare interface parameter used in the general forward_command_controller
    get_node()->undeclare_parameter("interface_name");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointGroupPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/position");
  }

  return state_interfaces_config;
}

CallbackReturn JointGroupPositionController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_configure(previous_state);

  fprintf(stderr, "got %zu joints\n", joint_names_.size());
  pids_.resize(joint_names_.size());
  std::string gains_prefix = "gains";
  for (auto k = 0u; k < joint_names_.size(); ++k) {
    auto p = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".p").as_double();
    auto i = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".i").as_double();
    auto d = get_node()->get_parameter(gains_prefix + "." + joint_names_[k] + ".d").as_double();
    pids_[k].initPid(p, i, d, 0.0, 0.0);
    fprintf(stderr, "got gains for %s as (%f, %f, %f)\n", joint_names_[k].c_str(), p, i, d);
  }

  return ret;
}

CallbackReturn JointGroupPositionController::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (command_interfaces_.size() != state_interfaces_.size()) {
    fprintf(stderr, "state interfaces don't match with command interfaces\n");
    return CallbackReturn::ERROR;
  }
  t0 = std::chrono::system_clock::now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointGroupPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  // stop all joints
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return ret;
}

controller_interface::return_type JointGroupPositionController::update()
{
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now() - t0).count();
  t0 = std::chrono::system_clock::now();

  auto joint_position_commands = *rt_command_ptr_.readFromRT();
  // no command received yet
  if (!joint_position_commands) {
    return controller_interface::return_type::OK;
  }

  if (joint_position_commands->data.size() != command_interfaces_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *node_->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      joint_position_commands->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for(auto i = 0u; i < joint_names_.size(); ++i)
  {
    double command_position = joint_position_commands->data[i];
    double current_position = state_interfaces_[i].get_value();
    auto error = angles::shortest_angular_distance(current_position, command_position);

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    auto commanded_effort = pids_[i].computeCommand(error, period);
    command_interfaces_[i].set_value(commanded_effort);

    // TODO(karsten1987):
    /*
     * enforce joint limits
     * calculate error terms depending on joint type
     */
  }

  t0 = std::chrono::system_clock::now();
  return controller_interface::return_type::OK;
}

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  effort_controllers::JointGroupPositionController, controller_interface::ControllerInterface)
