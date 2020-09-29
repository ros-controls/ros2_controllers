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

#include "forward_command_controller/forward_command_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace forward_command_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ForwardCommandController::ForwardCommandController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{}

CallbackReturn ForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rclcpp::Parameter joints_param, interface_param;
  if (!lifecycle_node_->get_parameter("joints", joint_names_)) {
    RCLCPP_ERROR_STREAM(get_lifecycle_node()->get_logger(), "'joints' parameter not set");
    return CallbackReturn::ERROR;
  }
  // TODO(anyone): here should be list of interface_names and they should be defined for every joint
  std::string interface_name;
  if (!lifecycle_node_->get_parameter("interface_name", interface_name)) {
    RCLCPP_ERROR_STREAM(get_lifecycle_node()->get_logger(), "'interface_name' parameter not set");
    return CallbackReturn::ERROR;
  }
  // TODO(anyone): a vector should be recived directyl from the parameter server.
  interfaces_.push_back(interface_name);

  joints_command_subscriber_ = lifecycle_node_->create_subscription<CmdType>(
    "commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      rt_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO_STREAM(get_lifecycle_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    for (const auto & interface : interfaces_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForwardCommandController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
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

  if ((*joint_commands)->data.size() != command_interfaces_.size()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_lifecycle_node()->get_logger(),
      *lifecycle_node_->get_clock(), 1000,
      "command size (" << (*joint_commands)->data.size() << ") does not match \
      number of interfaces (" << command_interfaces_.size() << ")");
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
