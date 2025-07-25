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

#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"

#include <memory>
#include <string>

#include "imu_sensor_broadcaster/imu_transform.hpp"

namespace imu_sensor_broadcaster
{
controller_interface::CallbackReturn IMUSensorBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    params_ = param_listener_->get_params();
    r_ =
      quat_from_euler(params_.calibration.roll, params_.calibration.pitch, params_.calibration.yaw);
    r_.normalize();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during config stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
    semantic_components::IMUSensor(params_.sensor_name));
  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  state_message_.header.frame_id = params_.frame_id;
  // convert double vector to fixed-size array in the message
  for (size_t i = 0; i < 9; ++i)
  {
    state_message_.orientation_covariance[i] = params_.static_covariance_orientation[i];
    state_message_.angular_velocity_covariance[i] = params_.static_covariance_angular_velocity[i];
    state_message_.linear_acceleration_covariance[i] =
      params_.static_covariance_linear_acceleration[i];
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IMUSensorBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration IMUSensorBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = imu_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  imu_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type IMUSensorBroadcaster::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::Imu input_imu;
  imu_sensor_->get_values_as_message(input_imu);
  doTransform(input_imu, r_, state_message_);

  if (realtime_publisher_)
  {
    state_message_.header.stamp = time;
    realtime_publisher_->try_publish(state_message_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type IMUSensorBroadcaster::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> IMUSensorBroadcaster::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_state_interfaces;

  std::string export_prefix = get_node()->get_name();
  if (!params_.sensor_name.empty())
  {
    // Update the prefix and get the proper IMU sensor naming
    export_prefix = export_prefix + "/" + params_.sensor_name;
  }

  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.x", &state_message_.orientation.x));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.y", &state_message_.orientation.y));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.z", &state_message_.orientation.z));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "orientation.w", &state_message_.orientation.w));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "angular_velocity.x", &state_message_.angular_velocity.x));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "angular_velocity.y", &state_message_.angular_velocity.y));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "angular_velocity.z", &state_message_.angular_velocity.z));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "linear_acceleration.x", &state_message_.linear_acceleration.x));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "linear_acceleration.y", &state_message_.linear_acceleration.y));
  exported_state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      export_prefix, "linear_acceleration.z", &state_message_.linear_acceleration.z));

  return exported_state_interfaces;
}

}  // namespace imu_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imu_sensor_broadcaster::IMUSensorBroadcaster, controller_interface::ChainableControllerInterface)
