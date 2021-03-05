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

#include "force_torque_sensor_controller/force_torque_sensor_controller.hpp"

#include <string>
#include <vector>

namespace force_torque_sensor_controller
{

ForceTorqueSensorController::ForceTorqueSensorController()
: controller_interface::ControllerInterface(),
  sensor_state_publisher_(nullptr)
{}

controller_interface::return_type
ForceTorqueSensorController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  try {
    auto node = get_node();
    node->declare_parameter<std::string>("sensor_name", "");
    node->declare_parameter<std::vector<std::string>>("interface_names", {});
    node->declare_parameter<std::string>("frame_id", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn ForceTorqueSensorController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = node_->get_parameter("sensor_name").as_string();
  if (sensor_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  interface_names_ = node_->get_parameter("interface_names").as_string_array();
  if (interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter was empty");
    return CallbackReturn::ERROR;
  }

  frame_id_ = node_->get_parameter("frame_id").as_string();
  if (frame_id_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter was empty");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");

  try {
    // register ft sensor data publisher
    publisher_ = node_->create_publisher<ControllerStateMsg>(
      "force_torque_sensor", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<StatePublisher>(publisher_);
  } catch (...) {
    return CallbackReturn::ERROR;
  }

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = frame_id_;
  state_publisher_->unlock();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & interface : interface_names_) {
    state_interfaces_config.names.push_back(sensor_name_ + "/" + interface);
  }

  return state_interfaces_config;
}

CallbackReturn ForceTorqueSensorController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueSensorController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueSensorController::update()
{
  if (state_interfaces_.size() != interface_names_.size()) {
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < state_interfaces_.size(); ++index) {
    switch (index) {
      case 0:
        wrench_state_msg_.wrench.force.x = state_interfaces_[index].get_value();
        break;
      case 1:
        wrench_state_msg_.wrench.torque.z = state_interfaces_[index].get_value();
        break;
      default:
        break;
    }
  }

  // publish
  sensor_state_publisher_->publish(wrench_state_msg_);
  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = node_->now();
    state_publisher_->msg_.wrench.force = wrench_state_msg_.wrench.force;
    state_publisher_->msg_.wrench.torque = wrench_state_msg_.wrench.torque;

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::SUCCESS;
}

}  // namespace force_torque_sensor_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_torque_sensor_controller::ForceTorqueSensorController,
  controller_interface::ControllerInterface)
