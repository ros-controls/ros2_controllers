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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace
{  // utility

// called from RT control loop
void reset_controller_reference_msg(forward_command_controller::CmdType & msg)
{
  for (auto & data : msg.data)
  {
    data = std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

namespace forward_command_controller
{
ForwardControllersBase::ForwardControllersBase()
: controller_interface::ControllerInterface(), joints_command_subscriber_(nullptr)
{
}

controller_interface::CallbackReturn ForwardControllersBase::on_init()
{
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

controller_interface::CallbackReturn ForwardControllersBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      const auto cmd = *msg;

      if (!std::all_of(
            cmd.data.cbegin(), cmd.data.cend(),
            [](const auto & value) { return std::isfinite(value); }))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Non-finite value received. Dropping message");
        return;
      }
      rt_command_.set(cmd);
    });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ForwardControllersBase::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardControllersBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  // Try to set default value in command.
  // If this fails, then another command will be received soon anyways.
  reset_controller_reference_msg(joint_commands_);
  rt_command_.try_set(joint_commands_);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Try to set default value in command.
  reset_controller_reference_msg(joint_commands_);
  rt_command_.try_set(joint_commands_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardControllersBase::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands_op = rt_command_.try_get();
  if (joint_commands_op.has_value())
  {
    joint_commands_ = joint_commands_op.value();
  }

  // no command received yet
  if (std::all_of(
        joint_commands_.data.cbegin(), joint_commands_.data.cend(),
        [](const auto & value) { return std::isnan(value); }))
  {
    return controller_interface::return_type::OK;
  }

  if (joint_commands_.data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)", joint_commands_.data.size(),
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  const auto & data = (*joint_commands)->data;

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    if (!command_interfaces_[index].set_value(data[index]))
    {
      RCLCPP_WARN(
        logger, "Unable to set the command interface value at index %zu: value = %f", index,
        data[index]);
      return controller_interface::return_type::OK;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller
