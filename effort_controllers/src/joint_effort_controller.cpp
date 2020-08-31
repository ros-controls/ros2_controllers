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

#include "effort_controllers/joint_effort_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace
{
constexpr auto kJECLoggerName = "joint effort controller";
}

namespace effort_controllers
{
using CallbackReturn = JointEffortController::CallbackReturn;

JointEffortController::JointEffortController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = kJECLoggerName;
}

CallbackReturn JointEffortController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  rclcpp::Parameter interface_param;
  if (!lifecycle_node_->get_parameter("interface_name", interface_param)) {
    lifecycle_node_->declare_parameter("interface_name", "effort_command");
  } else {
    if (interface_param.as_string() != "effort_command") {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          kJECLoggerName), "'interface_name' already set with an invalid value");
      return CallbackReturn::ERROR;
    }
  }
  return ForwardCommandController::on_configure(previous_state);
}

CallbackReturn JointEffortController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  // stop all joints
  for (auto & joint_handle : joint_cmd_handles_) {
    joint_handle.set_value(0.0);
  }

  return ret;
}

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  effort_controllers::JointEffortController, controller_interface::ControllerInterface)
