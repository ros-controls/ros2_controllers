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
    node->declare_parameter<std::vector<std::string>>("state_interface_names", {});
    node->declare_parameter<std::string>("frame_id", "");
    node->declare_parameter<int>("fx_range", -1);
    node->declare_parameter<int>("tz_range", -1);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn ForceTorqueSensorController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_interface_names_ = node_->get_parameter("state_interface_names").as_string_array();
  if (state_interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface_names' parameter was empty");
    return CallbackReturn::ERROR;
  }

  sensor_name_ = node_->get_parameter("sensor_name").as_string();
  if (sensor_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  frame_id_ = node_->get_parameter("frame_id").as_string();
  if (frame_id_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fx_range_ = node_->get_parameter("fx_range").as_int();
  if (fx_range_ == -1) {
    RCLCPP_ERROR(get_node()->get_logger(), "'fx_range' parameter was empty");
    return CallbackReturn::ERROR;
  }

  tz_range_ = node_->get_parameter("tz_range").as_int();
  if (tz_range_ == -1) {
    RCLCPP_ERROR(get_node()->get_logger(), "'tz_range' parameter was empty");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");

  try {
    // register ft sensor data publisher
    sensor_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "force_torque_sensor", rclcpp::SystemDefaultsQoS());
  } catch (...) {
    return CallbackReturn::ERROR;
  }

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

  for (const auto & interface : state_interface_names_) {
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
  geometry_msgs::msg::Vector3 f_vec;
  geometry_msgs::msg::Vector3 t_vec;

  if (state_interfaces_.size() != state_interface_names_.size())
    return controller_interface::return_type::ERROR;

  for (auto index = 0ul; index < state_interfaces_.size(); ++index) {
    switch (index) {
      case 0:
        f_vec.set__x(state_interfaces_[index].get_value());
        break;
      case 1:
        t_vec.set__z(state_interfaces_[index].get_value());
        break;
      default:
        break;
    }
  }

  wrench_state_msg_.header.stamp = get_node()->get_clock()->now();
  wrench_state_msg_.header.frame_id = frame_id_;

  // update wrench state message
  wrench_state_msg_.wrench.set__force(f_vec);
  wrench_state_msg_.wrench.set__torque(t_vec);

  // publish
  sensor_state_publisher_->publish(wrench_state_msg_);

  return controller_interface::return_type::SUCCESS;
}

}  // namespace force_torque_sensor_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_torque_sensor_controller::ForceTorqueSensorController,
  controller_interface::ControllerInterface)
