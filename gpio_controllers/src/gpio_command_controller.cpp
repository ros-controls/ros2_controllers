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
using hardware_interface::LoanedStateInterface;

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
  if (validate_configured_interfaces() != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  initialize_gpio_state_msg();
  rt_command_ptr_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GpioCommandController::validate_configured_interfaces()
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

  std::vector<std::reference_wrapper<LoanedStateInterface>> state_interface_order;
  if (
    !controller_interface::get_ordered_interfaces(
      state_interfaces_, interface_types_, std::string(""), state_interface_order) ||
    state_interfaces_.size() != state_interface_order.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu state interfaces, got %zu", interface_types_.size(),
      ordered_interfaces.size());

    for (const auto & interface : interface_types_)
      RCLCPP_ERROR(get_node()->get_logger(), "Got %s", interface.c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void GpioCommandController::initialize_gpio_state_msg()
{
  auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
  gpio_state_msg.header.stamp = get_node()->now();
  gpio_state_msg.joint_names.resize(params_.gpios.size());
  gpio_state_msg.interface_values.resize(params_.gpios.size());

  for (std::size_t gpio_index = 0; gpio_index < params_.gpios.size(); ++gpio_index)
  {
    const auto gpio_name = params_.gpios[gpio_index];
    gpio_state_msg.joint_names[gpio_index] = gpio_name;
    std::copy(
      params_.command_interfaces.gpios_map[gpio_name].ports.begin(),
      params_.command_interfaces.gpios_map[gpio_name].ports.end(),
      std::back_insert_iterator(gpio_state_msg.interface_values[gpio_index].interface_names));
    gpio_state_msg.interface_values[gpio_index].values = std::vector<double>(
      params_.command_interfaces.gpios_map[gpio_name].ports.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
}

CallbackReturn GpioCommandController::on_deactivate(const rclcpp_lifecycle::State &)
{
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GpioCommandController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  update_gpios_states();
  return update_gpios_commands();
}

controller_interface::return_type GpioCommandController::update_gpios_commands()
{
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

  for (std::size_t command_index = 0; command_index < command_interfaces_.size(); ++command_index)
  {
    command_interfaces_[command_index].set_value((*gpio_commands)->data[command_index]);
  }
  return controller_interface::return_type::OK;
}

void GpioCommandController::update_gpios_states()
{
  if (realtime_gpio_state_publisher_ && realtime_gpio_state_publisher_->trylock())
  {
    auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
    gpio_state_msg.header.stamp = get_node()->now();

    std::size_t sindex = 0;
    for (std::size_t g = 0; g < params_.gpios.size(); g++)
    {
      for (auto & interface_value : gpio_state_msg.interface_values[g].values)
      {
        interface_value = state_interfaces_[sindex].get_value();
        sindex++;
      }
    }
    realtime_gpio_state_publisher_->unlockAndPublish();
  }
}

}  // namespace gpio_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gpio_controllers::GpioCommandController, controller_interface::ControllerInterface)
