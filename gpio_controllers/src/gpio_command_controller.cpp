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

namespace
{
void print_interface(
  const rclcpp::Logger & logger, const gpio_controllers::StateInterfaces & state_interfaces)
{
  for (const auto & interface : state_interfaces)
  {
    RCLCPP_ERROR(logger, "Got %s", interface.get().get_name().c_str());
  }
}
void print_interface(
  const rclcpp::Logger & logger,
  const gpio_controllers::MapOfReferencesToCommandInterfaces & command_interfaces)
{
  for (const auto & [interface_name, value] : command_interfaces)
  {
    RCLCPP_ERROR(logger, "Got %s", interface_name.c_str());
  }
}
}  // namespace

namespace gpio_controllers
{

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
  auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

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
  command_interfaces_map_ = create_map_of_references_to_interfaces(interface_types_);
  controller_interface::get_ordered_interfaces(
    state_interfaces_, interface_types_, "", ordered_state_interfaces_);

  if (
    !check_if_configured_interfaces_matches_received(interface_types_, command_interfaces_map_) ||
    !check_if_configured_interfaces_matches_received(interface_types_, ordered_state_interfaces_))
  {
    return CallbackReturn::ERROR;
  }

  initialize_gpio_state_msg();
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
  update_gpios_states();
  return update_gpios_commands();
}

void GpioCommandController::store_interface_types()
{
  for (const auto & [gpio_name, interfaces] : params_.command_interfaces.gpios_map)
  {
    std::transform(
      interfaces.interfaces.begin(), interfaces.interfaces.end(),
      std::back_inserter(interface_types_),
      [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
  }
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
      params_.command_interfaces.gpios_map[gpio_name].interfaces.begin(),
      params_.command_interfaces.gpios_map[gpio_name].interfaces.end(),
      std::back_insert_iterator(gpio_state_msg.interface_values[gpio_index].interface_names));
    gpio_state_msg.interface_values[gpio_index].values = std::vector<double>(
      params_.command_interfaces.gpios_map[gpio_name].interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
}

MapOfReferencesToCommandInterfaces GpioCommandController::create_map_of_references_to_interfaces(
  const InterfacesNames & interfaces_from_params)
{
  MapOfReferencesToCommandInterfaces map;
  for (const auto & interface_name : interfaces_from_params)
  {
    auto interface = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & configured_interface)
      {
        const auto full_name_interface_name = configured_interface.get_name();
        return full_name_interface_name == interface_name;
      });
    if (interface != command_interfaces_.end())
    {
      map.emplace(interface_name, std::ref(*interface));
    }
  }
  return map;
}

template <typename T>
bool GpioCommandController::check_if_configured_interfaces_matches_received(
  const InterfacesNames & interfaces_from_params, const T & configured_interfaces)
{
  if (!(configured_interfaces.size() == interfaces_from_params.size()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %ld interfaces, got %ld", interfaces_from_params.size(),
      configured_interfaces.size());
    for (const auto & interface : interfaces_from_params)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %s", interface.c_str());
    }
    print_interface(get_node()->get_logger(), configured_interfaces);
    return false;
  }
  return true;
}

controller_interface::return_type GpioCommandController::update_gpios_commands()
{
  auto gpio_commands_ptr = rt_command_ptr_.readFromRT();
  if (!gpio_commands_ptr || !(*gpio_commands_ptr))
  {
    return controller_interface::return_type::OK;
  }

  const auto gpio_commands = *(*gpio_commands_ptr);
  for (std::size_t gpio_index = 0; gpio_index < gpio_commands.joint_names.size(); ++gpio_index)
  {
    const auto & gpio_name = gpio_commands.joint_names[gpio_index];
    if (
      gpio_commands.interface_values[gpio_index].values.size() !=
      gpio_commands.interface_values[gpio_index].interface_names.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "For gpio %s interfaces_names do not match values",
        gpio_name.c_str());
      return controller_interface::return_type::ERROR;
    }

    for (std::size_t command_interface_index = 0;
         command_interface_index < gpio_commands.interface_values[gpio_index].values.size();
         ++command_interface_index)
    {
      const auto & full_command_interface_name =
        gpio_name + '/' +
        gpio_commands.interface_values[gpio_index].interface_names[command_interface_index];
      try
      {
        command_interfaces_map_.at(full_command_interface_name)
          .get()
          .set_value(gpio_commands.interface_values[gpio_index].values[command_interface_index]);
      }
      catch (const std::exception & e)
      {
        fprintf(
          stderr, "Exception thrown during applying command stage of %s with message: %s \n",
          full_command_interface_name, e.what());
      }
    }
  }
  return controller_interface::return_type::OK;
}

void GpioCommandController::update_gpios_states()
{
  if (realtime_gpio_state_publisher_ && realtime_gpio_state_publisher_->trylock())
  {
    auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
    gpio_state_msg.header.stamp = get_node()->now();

    std::size_t state_index{};
    for (std::size_t gpio_index = 0; gpio_index < gpio_state_msg.joint_names.size(); ++gpio_index)
    {
      for (std::size_t interface_index = 0;
           interface_index < gpio_state_msg.interface_values[gpio_index].interface_names.size();
           ++interface_index)
      {
        gpio_state_msg.interface_values[gpio_index].values[interface_index] =
          ordered_state_interfaces_[state_index].get().get_value();
        ++state_index;
      }
    }
    realtime_gpio_state_publisher_->unlockAndPublish();
  }
}

}  // namespace gpio_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gpio_controllers::GpioCommandController, controller_interface::ControllerInterface)
