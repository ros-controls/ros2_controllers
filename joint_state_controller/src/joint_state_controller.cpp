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
#include <vector>

#include "hardware_interface/joint_handle.hpp"
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
static const auto kPositionIndex = 0ul;
static const auto kVelocityIndex = 1ul;
static const auto kEffortIndex = 2ul;

JointStateController::JointStateController()
: controller_interface::ControllerInterface()
{}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointStateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  init_joint_state_msg();
  init_dynamic_joint_state_msg();

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

bool JointStateController::init_joint_data()
{
  if (auto rh_ptr = robot_hardware_.lock()) {
    joint_names_ = rh_ptr->get_registered_joint_names();

    if (joint_names_.empty()) {
      return false;
    }

    // reset any previous joint data
    joint_handles_.clear();

    for (const auto & joint_name : joint_names_) {
      // always check for these interfaces, even if they are not supported by the joint
      std::vector<std::string> interface_names = {"position", "velocity", "effort"};
      const std::vector<std::string> joint_interface_names =
        rh_ptr->get_registered_joint_interface_names(joint_name);

      // add the rest of the interfaces
      // ideally this could be done with std::sort + std::unique, but we rather have
      // position, velocity and effort in known indexes
      std::copy_if(
        joint_interface_names.begin(), joint_interface_names.end(),
        std::back_inserter(interface_names), [&](const auto & value) {
          return std::find(
            interface_names.begin(),
            interface_names.end(), value) == interface_names.end();
        });

      std::vector<std::shared_ptr<hardware_interface::JointHandle>> joint_interface_handles;
      for (const auto & interface_name : interface_names) {
        /// @todo is there a better way of skipping command interfaces?
        if (interface_name.find("command") != std::string::npos) {
          continue;
        }

        auto joint_handle = std::make_shared<hardware_interface::JointHandle>(
          joint_name,
          interface_name);
        if (rh_ptr->get_joint_handle(*joint_handle) == hardware_interface::return_type::OK) {
          joint_interface_handles.emplace_back(joint_handle);
        } else {
          // leave a nullptr for not supported interfaces
          joint_interface_handles.emplace_back(nullptr);
        }
      }
      joint_handles_.emplace_back(joint_interface_handles);
    }
    return true;
  } else {
    return false;
  }
}

void JointStateController::init_joint_state_msg()
{
  const size_t num_joints = joint_handles_.size();

  /// @note joint_state_msg publishes position, velocity and effort for all joints,
  /// regardless of the joints actually supporting these interfaces

  // default initialization for joint state message
  joint_state_msg_.name = joint_names_;
  joint_state_msg_.position.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_msg_.velocity.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_msg_.effort.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
}

void JointStateController::init_dynamic_joint_state_msg()
{
  dynamic_joint_state_msg_.joint_names = joint_names_;
  for (const auto & joint_handle : joint_handles_) {
    control_msgs::msg::InterfaceValue if_value;
    for (const auto & joint_interface_handle : joint_handle) {
      // check joint handle is valid
      if (joint_interface_handle) {
        if_value.interface_names.emplace_back(joint_interface_handle->get_interface_name());
        if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
      }
    }
    dynamic_joint_state_msg_.interface_values.emplace_back(if_value);
  }
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

  joint_state_msg_.header.stamp = lifecycle_node_->get_clock()->now();

  // update joint state message and dynamic joint state message
  for (auto joint_index = 0ul; joint_index < joint_names_.size(); ++joint_index) {
    auto update_value =
      [](double & msg_data, const std::shared_ptr<hardware_interface::JointHandle> & joint_handle)
      {
        if (joint_handle) {
          msg_data = joint_handle->get_value();
        }
      };

    // the interface indexes used here are hardcoded, check init_joint_data()
    update_value(
      joint_state_msg_.position[joint_index],
      joint_handles_[joint_index][kPositionIndex]);
    update_value(
      joint_state_msg_.velocity[joint_index],
      joint_handles_[joint_index][kVelocityIndex]);
    update_value(
      joint_state_msg_.effort[joint_index],
      joint_handles_[joint_index][kEffortIndex]);

    auto interface_index = 0ul;
    for (const auto & joint_interface_handle : joint_handles_[joint_index]) {
      if (joint_interface_handle) {
        dynamic_joint_state_msg_.interface_values[joint_index].values[interface_index] =
          joint_interface_handle->get_value();
        ++interface_index;
      }
    }
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
