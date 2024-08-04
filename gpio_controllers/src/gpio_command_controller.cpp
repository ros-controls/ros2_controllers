// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "gpio_controllers/gpio_command_controller.hpp"

#include <algorithm>

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"

namespace gpio_controllers
{
using hardware_interface::LoanedCommandInterface;

GpioCommandController::GpioCommandController() : controller_interface::ControllerInterface() {}

CallbackReturn GpioCommandController::on_init()
try
{
  param_listener_ = std::make_shared<gpio_command_controller_parameters::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}

CallbackReturn GpioCommandController::on_configure(const rclcpp_lifecycle::State &)
try
{
  store_interface_types();
  gpios_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  gpio_state_publisher_ =
    get_node()->create_publisher<StateType>("~/gpio_states", rclcpp::SystemDefaultsQoS());

  realtime_gpio_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StateType>>(gpio_state_publisher_);
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}

void GpioCommandController::store_interface_types()
{
  for (const auto & [gpio_name, ports] : params_.command_interfaces.gpios_map)
  {
    std::transform(
      ports.ports.begin(), ports.ports.end(), std::back_inserter(interface_types_),
      [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
  }
}

controller_interface::InterfaceConfiguration
GpioCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration GpioCommandController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = interface_types_;

  return state_interfaces_config;
}

CallbackReturn GpioCommandController::on_activate(const rclcpp_lifecycle::State &)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, interface_types_, std::string(""), ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu", interface_types_.size(),
      ordered_interfaces.size());

    for (const auto & interface : interface_types_)
      RCLCPP_ERROR(get_node()->get_logger(), "Got %s", interface.c_str());
    return CallbackReturn::ERROR;
  }

  auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
  gpio_state_msg.header.stamp = get_node()->now();
  gpio_state_msg.joint_names.resize(params_.gpios.size());
  gpio_state_msg.interface_values.resize(params_.gpios.size());

  for (auto g = 0ul; g < params_.gpios.size(); g++)
  {
    gpio_state_msg.joint_names[g] = params_.gpios[g];
    for (const auto & interface_name : params_.command_interfaces.gpios_map[params_.gpios[g]].ports)
    {
      gpio_state_msg.interface_values[g].interface_names.push_back(interface_name);
      gpio_state_msg.interface_values[g].values.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }
  rt_command_ptr_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GpioCommandController::on_deactivate(const rclcpp_lifecycle::State &)
{
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GpioCommandController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (realtime_gpio_state_publisher_ && realtime_gpio_state_publisher_->trylock())
  {
    auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
    gpio_state_msg.header.stamp = get_node()->now();

    auto sindex = 0ul;
    for (auto g = 0ul; g < params_.gpios.size(); g++)
    {
      for (auto i = 0ul; i < params_.command_interfaces.gpios_map[params_.gpios[g]].ports.size();
           i++)
      {
        gpio_state_msg.interface_values[g].values[i] = state_interfaces_[sindex].get_value();
        sindex++;
      }
    }

    realtime_gpio_state_publisher_->unlockAndPublish();
  }

  auto gpio_commands = rt_command_ptr_.readFromRT();

  if (!gpio_commands || !(*gpio_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*gpio_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)", (*gpio_commands)->data.size(),
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t cindex = 0; cindex < command_interfaces_.size(); ++cindex)
  {
    command_interfaces_[cindex].set_value((*gpio_commands)->data[cindex]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace gpio_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gpio_controllers::GpioCommandController, controller_interface::ControllerInterface)
