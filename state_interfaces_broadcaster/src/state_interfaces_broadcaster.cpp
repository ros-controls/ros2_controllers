// Copyright (c) 2025, PAL Robotics
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

#include "state_interfaces_broadcaster/state_interfaces_broadcaster.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"

namespace state_interfaces_broadcaster
{

StateInterfacesBroadcaster::StateInterfacesBroadcaster() {}

controller_interface::CallbackReturn StateInterfacesBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return params_.interfaces.empty() ? CallbackReturn::ERROR : CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StateInterfacesBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
StateInterfacesBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & interface : params_.interfaces)
  {
    state_interfaces_config.names.push_back(interface);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn StateInterfacesBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  names_publisher_ = get_node()->create_publisher<control_msgs::msg::Keys>(
    "~/names", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  values_publisher_ = get_node()->create_publisher<control_msgs::msg::Float64Values>(
    "~/values", rclcpp::SystemDefaultsQoS());
  realtime_values_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::Float64Values>>(
      values_publisher_);

  values_msg_.values.clear();
  values_msg_.values.resize(params_.interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  names_msg_.keys = params_.interfaces;
  names_msg_.header.stamp = get_node()->now();
  names_publisher_->publish(names_msg_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StateInterfacesBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < state_interfaces_.size(); ++i)
  {
    if (state_interfaces_[i].get_data_type() != hardware_interface::HandleDataType::DOUBLE)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "State interface '%s' (%s) is not of type double. The StateInterfacesBroadcaster only "
        "supports state interfaces that support double datatype.",
        params_.interfaces[i].c_str(), state_interfaces_[i].get_data_type().to_string().c_str());
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StateInterfacesBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (auto i = 0u; i < state_interfaces_.size(); ++i)
  {
    const auto & opt = state_interfaces_[i].get_optional(0);
    if (opt.has_value())
    {
      values_msg_.values[i] = opt.value();
    }
  }

  if (realtime_values_publisher_)
  {
    values_msg_.header.stamp = time;
    realtime_values_publisher_->try_publish(values_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace state_interfaces_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  state_interfaces_broadcaster::StateInterfacesBroadcaster,
  controller_interface::ControllerInterface)
