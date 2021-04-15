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
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace forward_command_controller
{
using hardware_interface::LoanedCommandInterface;

ForwardCommandController::ForwardCommandController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{}

controller_interface::return_type
ForwardCommandController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto node = get_node();
    node->declare_parameter<std::vector<std::string>>("joints", {});

    node->declare_parameter<std::string>("interface_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn ForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = node_->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling init function.
  if (interface_name_.empty()) {
    interface_name_ = node_->get_parameter("interface_name").as_string();
  }

  if (interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      rt_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForwardCommandController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if ((command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn ForwardCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (!get_ordered_interfaces(
      command_interfaces_, joint_names_, interface_name_,
      ordered_interfaces) || command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position command interfaces, got %u",
      joint_names_.size(), ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

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
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_node()->get_logger(),
      *node_->get_clock(), 1000,
      "command size (" << (*joint_commands)->data.size() <<
        ") does not match number of interfaces (" <<
        command_interfaces_.size() << ")");
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
