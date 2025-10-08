// Copyright 2021 PAL Robotics SL.
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

/*
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "range_sensor_broadcaster/range_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace range_sensor_broadcaster
{
controller_interface::CallbackReturn RangeSensorBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RangeSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (params_.sensor_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  if (params_.frame_id.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be provided.");
    return CallbackReturn::ERROR;
  }

  range_sensor_ = std::make_unique<semantic_components::RangeSensor>(params_.sensor_name);
  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Range>("~/range", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  range_msg_.header.frame_id = params_.frame_id;
  range_msg_.radiation_type = static_cast<uint8_t>(params_.radiation_type);
  range_msg_.field_of_view = static_cast<float>(params_.field_of_view);
  range_msg_.min_range = static_cast<float>(params_.min_range);
  range_msg_.max_range = static_cast<float>(params_.max_range);
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  range_msg_.variance = params_.variance;
#endif

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RangeSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RangeSensorBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = range_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn RangeSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  range_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RangeSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  range_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RangeSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_)
  {
    range_msg_.header.stamp = time;
    range_sensor_->get_values_as_message(range_msg_);
    realtime_publisher_->try_publish(range_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace range_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  range_sensor_broadcaster::RangeSensorBroadcaster, controller_interface::ControllerInterface)
