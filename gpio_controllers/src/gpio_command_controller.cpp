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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace gpio_controllers
{
using hardware_interface::LoanedCommandInterface;

GpioCommandController::GpioCommandController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  gpios_command_subscriber_(nullptr)
{
}

CallbackReturn GpioCommandController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("gpios", std::vector<std::string>());
    gpio_names_ = get_node()->get_parameter("gpios").as_string_array();
    for(std::string &gpio : gpio_names_)
      auto_declare<std::vector<std::string>>("command_interfaces."
          + gpio, std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn GpioCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    gpio_names_ = get_node()->get_parameter("gpios").as_string_array();

    if (gpio_names_.empty()){
        RCLCPP_ERROR(get_node()->get_logger(), "'gpios' parameter was empty");
        return CallbackReturn::ERROR;
    }

    for(std::string &gpio : gpio_names_){
      auto interfaces = get_node()->get_parameter("command_interfaces." + gpio).as_string_array();
      if (interfaces.empty()){
        RCLCPP_ERROR(get_node()->get_logger(),
            "'command_interfaces.%s' parameter was empty", gpio.c_str());
        return CallbackReturn::ERROR;
      }
      if ( !interface_names_.insert( std::make_pair( gpio, interfaces) ).second ) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Trying to override existing gpio setup. Wrong controller parameters.");
        return CallbackReturn::ERROR;
      }
    }

    for(const auto & gpio : gpio_names_){
      for(const auto & interface_name: interface_names_[gpio]){
        interface_types_.push_back(gpio + "/" + interface_name);
      }
    }

    try
    {
      gpios_command_subscriber_ = get_node()->create_subscription<CmdType>(
          "~/commands", rclcpp::SystemDefaultsQoS(),
          [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

      gpio_state_publisher_ =
        get_node()->create_publisher<StateType>(
          "~/gpio_states", rclcpp::SystemDefaultsQoS());

      realtime_gpio_state_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<StateType>>(
          gpio_state_publisher_);
    }
    catch (const std::exception & e)
    {
      // get_node() may throw, logging raw here
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GpioCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
GpioCommandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = interface_types_;

  return state_interfaces_config;
}

CallbackReturn GpioCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, interface_types_, std::string(""), ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu", interface_types_.size(),
      ordered_interfaces.size());

    for(const auto & interface: interface_types_)
      RCLCPP_ERROR(get_node()->get_logger(), "Got %s", interface.c_str());
    return CallbackReturn::ERROR;
  }

  //initialize gpio state msg
  auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
  for(auto i = 0ul; i<interface_types_.size();i++){
    gpio_state_msg.interface_names.push_back(interface_types_[i]);
    gpio_state_msg.values.push_back(std::numeric_limits<double>::quiet_NaN());
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn GpioCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GpioCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //publish gpio state msg
  if (realtime_gpio_state_publisher_ && realtime_gpio_state_publisher_->trylock())
  {
    auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;

    for (size_t iindex = 0; iindex < state_interfaces_.size(); ++iindex){
      gpio_state_msg.values[iindex] = state_interfaces_[iindex].get_value();
    }
    realtime_gpio_state_publisher_->unlockAndPublish();
  } 
  
  auto gpio_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!gpio_commands || !(*gpio_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*gpio_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*gpio_commands)->data.size(), command_interfaces_.size());
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
