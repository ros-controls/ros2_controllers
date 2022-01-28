// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
 * Authors: Subhas Das, Denis Stogl
 */

#include "force_torque_sensor_broadcaster/force_torque_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace force_torque_sensor_broadcaster
{
ForceTorqueSensorBroadcaster::ForceTorqueSensorBroadcaster()
: controller_interface::ControllerInterface()
{
}

CallbackReturn ForceTorqueSensorBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("interface_names.force.x", "");
    auto_declare<std::string>("interface_names.force.y", "");
    auto_declare<std::string>("interface_names.force.z", "");
    auto_declare<std::string>("interface_names.torque.x", "");
    auto_declare<std::string>("interface_names.torque.y", "");
    auto_declare<std::string>("interface_names.torque.z", "");
    auto_declare<std::string>("frame_id", "");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = node_->get_parameter("sensor_name").as_string();
  interface_names_[0] = node_->get_parameter("interface_names.force.x").as_string();
  interface_names_[1] = node_->get_parameter("interface_names.force.y").as_string();
  interface_names_[2] = node_->get_parameter("interface_names.force.z").as_string();
  interface_names_[3] = node_->get_parameter("interface_names.torque.x").as_string();
  interface_names_[4] = node_->get_parameter("interface_names.torque.y").as_string();
  interface_names_[5] = node_->get_parameter("interface_names.torque.z").as_string();

  const bool no_interface_names_defined =
    std::count(interface_names_.begin(), interface_names_.end(), "") == 6;

  if (sensor_name_.empty() && no_interface_names_defined)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "'sensor_name' or at least one "
      "'interface_names.[force|torque].[x|y|z]' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty() && !no_interface_names_defined)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "both 'sensor_name' and "
      "'interface_names.[force|torque].[x|y|z]' parameters can not be specified together.");
    return CallbackReturn::ERROR;
  }

  frame_id_ = node_->get_parameter("frame_id").as_string();
  if (frame_id_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "'frame_id' parameter has to be provided.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty())
  {
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(sensor_name_));
  }
  else
  {
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(
        interface_names_[0], interface_names_[1], interface_names_[2], interface_names_[3],
        interface_names_[4], interface_names_[5]));
  }

  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = frame_id_;
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(node_->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

CallbackReturn ForceTorqueSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    force_torque_sensor_->get_values_as_message(realtime_publisher_->msg_.wrench);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace force_torque_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_torque_sensor_broadcaster::ForceTorqueSensorBroadcaster,
  controller_interface::ControllerInterface)
