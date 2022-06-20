// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "forward_command_controller/forward_controllers_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ForwardControllersBase::ForwardControllersBase()
: rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr)
{
}

controller_interface::CallbackReturn ForwardControllersBase::execute_init(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  node_ = node;

  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::execute_configure(
  const rclcpp_lifecycle::State & /*previous_state*/,
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error when reading parameters.");
    return ret;
  }

  joints_command_subscriber_ = node_->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(), [this](const CmdType::SharedPtr msg) {
      // check if message is correct size, if not ignore
      if (msg->data.size() == command_interface_names_.size())
      {
        rt_command_ptr_.writeFromNonRT(msg);
      }
    });

  // pre-reserve command interfaces
  command_interfaces.reserve(command_interface_names_.size());

  RCLCPP_INFO(node_->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardControllersBase::get_command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForwardControllersBase::get_state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardControllersBase::execute_activate(
  const rclcpp_lifecycle::State & /*previous_state*/,
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(node_->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::execute_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller
