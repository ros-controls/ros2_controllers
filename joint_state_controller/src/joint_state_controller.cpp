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

#include <string>
#include <memory>

#include "rclcpp_lifecycle/state.hpp"

#include "rcutils/logging_macros.h"

namespace ros_controllers
{

JointStateController::JointStateController()
: controller_interface::ControllerInterface()
{}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointStateController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  if (auto sptr = robot_hardware_.lock()) {
    registered_joint_handles_ = sptr->get_registered_joint_state_handles();
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (registered_joint_handles_.empty()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  size_t num_joints = registered_joint_handles_.size();
  // default initialize joint state message
  joint_state_msg_.position.resize(num_joints);
  joint_state_msg_.velocity.resize(num_joints);
  joint_state_msg_.effort.resize(num_joints);
  // set known joint names
  joint_state_msg_.name.reserve(num_joints);
  for (auto joint_handle : registered_joint_handles_) {
    joint_state_msg_.name.push_back(joint_handle->get_name());
  }

  joint_state_publisher_ = lifecycle_node_->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS());
  joint_state_publisher_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::controller_interface_ret_t
JointStateController::update()
{
  if (!joint_state_publisher_->is_activated()) {
    RCUTILS_LOG_WARN_ONCE_NAMED("publisher", "joint state publisher is not activated");
    return hardware_interface::HW_RET_ERROR;
  }

  joint_state_msg_.header.stamp = rclcpp::Clock().now();
  size_t i = 0;
  for (auto joint_state_handle : registered_joint_handles_) {
    joint_state_msg_.position[i] = joint_state_handle->get_position();
    joint_state_msg_.velocity[i] = joint_state_handle->get_velocity();
    joint_state_msg_.effort[i] = joint_state_handle->get_effort();
    ++i;
  }

  // publish
  joint_state_publisher_->publish(joint_state_msg_);
  return hardware_interface::HW_RET_OK;
}

}  // namespace ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros_controllers::JointStateController, controller_interface::ControllerInterface)
