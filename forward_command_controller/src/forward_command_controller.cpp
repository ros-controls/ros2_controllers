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

#include <algorithm>
#include <utility>

#include "forward_command_controller/forward_command_controller.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr auto kFCCLoggerName = "forward command controller";
}

namespace forward_command_controller
{
using CallbackReturn = ForwardCommandController::CallbackReturn;

ForwardCommandController::ForwardCommandController()
: controller_interface::ControllerInterface(),
  joint_cmd_handles_(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{}

CallbackReturn ForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rclcpp::Parameter joints_param, interface_param;
  if (!lifecycle_node_->get_parameter("joints", joints_param)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(kFCCLoggerName), "'joints' parameter not set");
    return CallbackReturn::ERROR;
  }
  if (!lifecycle_node_->get_parameter("interface_name", interface_param)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(kFCCLoggerName), "'interface_name' parameter not set");
    return CallbackReturn::ERROR;
  }

  auto joint_names = joints_param.as_string_array();
  if (joint_names.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(kFCCLoggerName), "'joints' is empty");
    return CallbackReturn::ERROR;
  }

  auto interface_name = interface_param.as_string();
  if (interface_name.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(kFCCLoggerName), "'interface_name' is empty");
    return CallbackReturn::ERROR;
  }

  if (auto rh_ptr = robot_hardware_.lock()) {
    const auto registered_joints = rh_ptr->get_registered_joint_names();

    // check all requested joints are present
    for (const auto & joint_name : joint_names) {
      if (std::find(
          registered_joints.cbegin(), registered_joints.cend(),
          joint_name) == registered_joints.cend())
      {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            kFCCLoggerName), "joint '" << joint_name << "' not registered");
        return CallbackReturn::ERROR;
      }
    }

    // get joint handles
    for (const auto & joint_name : joint_names) {
      hardware_interface::JointHandle joint_handle(joint_name, interface_name);
      if (rh_ptr->get_joint_handle(joint_handle) ==
        hardware_interface::hardware_interface_ret_t::ERROR)
      {
        // uppon error, clear any previously requested handles
        joint_cmd_handles_.clear();

        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            kFCCLoggerName), "could not get handle for joint '" << joint_name << "'");
        return CallbackReturn::ERROR;
      }
      joint_cmd_handles_.emplace_back(std::move(joint_handle));
    }
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        kFCCLoggerName), "could not lock pointer to robot_hardware");
    return CallbackReturn::ERROR;
  }

  joints_command_subscriber_ = lifecycle_node_->create_subscription<CmdType>(
    "commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      rt_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      kFCCLoggerName), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForwardCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForwardCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardCommandController::update()
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands)) {
    return controller_interface::return_type::SUCCESS;
  }

  const auto joint_num = (*joint_commands)->data.size();
  if (joint_num != joint_cmd_handles_.size()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        kFCCLoggerName), "command size does not match number of joints");
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < joint_num; ++index) {
    joint_cmd_handles_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
