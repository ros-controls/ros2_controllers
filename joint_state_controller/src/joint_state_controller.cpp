// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "joint_state_controller/joint_state_controller.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>


#include "hardware_interface/joint_state_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_state_controller
{

JointStateController::JointStateController()
: controller_interface::ControllerInterface()
{}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointStateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (auto sptr = robot_hardware_.lock()) {
    registered_joint_handles_ = sptr->get_registered_joint_state_handles();
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (registered_joint_handles_.empty()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  const size_t num_joints = registered_joint_handles_.size();

  // default initialization for joint state message
  joint_state_msg_.position.resize(num_joints);
  joint_state_msg_.velocity.resize(num_joints);
  joint_state_msg_.effort.resize(num_joints);

  // default initialization for dynamic joint state message
  control_msgs::msg::InterfaceValue default_if_value;
  default_if_value.interface_names = {"position", "velocity", "effort"};
  default_if_value.values.resize(
    default_if_value.interface_names.size(), std::numeric_limits<double>::quiet_NaN());

  // set known joint names
  joint_state_msg_.name.reserve(num_joints);
  dynamic_joint_state_msg_.joint_names.reserve(num_joints);
  for (const auto joint_handle : registered_joint_handles_) {
    joint_state_msg_.name.push_back(joint_handle->get_name());

    dynamic_joint_state_msg_.joint_names.push_back(joint_handle->get_name());
    dynamic_joint_state_msg_.interface_values.push_back(default_if_value);
  }

  joint_state_publisher_ = lifecycle_node_->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS());

  dynamic_joint_state_publisher_ =
    lifecycle_node_->create_publisher<control_msgs::msg::DynamicJointState>(
    "dynamic_joint_states", rclcpp::SystemDefaultsQoS());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointStateController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_state_publisher_->on_activate();
  dynamic_joint_state_publisher_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointStateController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_state_publisher_->on_deactivate();
  dynamic_joint_state_publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
JointStateController::update()
{
  if (!joint_state_publisher_->is_activated()) {
    RCUTILS_LOG_WARN_ONCE_NAMED("publisher", "joint state publisher is not activated");
    return controller_interface::return_type::ERROR;
  }

  if (!dynamic_joint_state_publisher_->is_activated()) {
    RCUTILS_LOG_WARN_ONCE_NAMED("publisher", "dynamic joint state publisher is not activated");
    return controller_interface::return_type::ERROR;
  }

  joint_state_msg_.header.stamp = rclcpp::Clock().now();
  size_t i = 0;
  for (auto joint_state_handle : registered_joint_handles_) {
    joint_state_msg_.position[i] = joint_state_handle->get_position();
    joint_state_msg_.velocity[i] = joint_state_handle->get_velocity();
    joint_state_msg_.effort[i] = joint_state_handle->get_effort();

    dynamic_joint_state_msg_.interface_values[i].values[0] = joint_state_handle->get_position();
    dynamic_joint_state_msg_.interface_values[i].values[1] = joint_state_handle->get_velocity();
    dynamic_joint_state_msg_.interface_values[i].values[2] = joint_state_handle->get_effort();

    ++i;
  }

  // publish
  joint_state_publisher_->publish(joint_state_msg_);
  dynamic_joint_state_publisher_->publish(dynamic_joint_state_msg_);

  return controller_interface::return_type::SUCCESS;
}

}  // namespace joint_state_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_state_controller::JointStateController, controller_interface::ControllerInterface)
