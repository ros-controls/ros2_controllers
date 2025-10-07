// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#include "motion_primitives_controllers/motion_primitives_base_controller.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/helpers.hpp"

namespace motion_primitives_controllers
{
MotionPrimitivesBaseController::MotionPrimitivesBaseController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MotionPrimitivesBaseController::on_init()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Initializing Motion Primitives Base Controller");
  // needs to be implemented in derived classes
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesBaseController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Configuring Motion Primitives Base Controller");
  // needs to be implemented in derived classes
  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesBaseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(25);
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/motion_type");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q1");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q2");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q3");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q4");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q5");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/q6");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_x");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_y");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_z");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_qx");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_qy");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_qz");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_qw");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_x");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_y");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_z");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_qx");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_qy");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_qz");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/pos_via_qw");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/blend_radius");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/velocity");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/acceleration");
  command_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/move_time");
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesBaseController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(2);
  state_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/execution_status");
  state_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/ready_for_new_primitive");
  return state_interfaces_config;
}

controller_interface::CallbackReturn MotionPrimitivesBaseController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesBaseController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionPrimitivesBaseController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // This method should be implemented in derived classes
  RCLCPP_ERROR(get_node()->get_logger(), "Update method not implemented in derived class");
  return controller_interface::return_type::ERROR;
}

// Reset Command-Interfaces to nan
void MotionPrimitivesBaseController::reset_command_interfaces()
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset command interface %ld", i);
    }
  }
}

// Set command interfaces from the message, gets called in the update function
bool MotionPrimitivesBaseController::set_command_interfaces()
{
  // Get the oldest message from the queue
  if (!moprim_queue_.pop(current_moprim_))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to pop motion primitive from queue.");
    return false;
  }

  // Set the motion_type
  (void)command_interfaces_[0].set_value(static_cast<double>(current_moprim_.type));

  // Process joint positions if available
  if (!current_moprim_.joint_positions.empty())
  {
    for (size_t i = 0; i < current_moprim_.joint_positions.size(); ++i)
    {
      (void)command_interfaces_[i + 1].set_value(current_moprim_.joint_positions[i]);  // q1 to q6
    }
  }

  // Process Cartesian poses if available
  if (!current_moprim_.poses.empty())
  {
    const auto & goal_pose = current_moprim_.poses[0].pose;            // goal pose
    (void)command_interfaces_[7].set_value(goal_pose.position.x);      // pos_x
    (void)command_interfaces_[8].set_value(goal_pose.position.y);      // pos_y
    (void)command_interfaces_[9].set_value(goal_pose.position.z);      // pos_z
    (void)command_interfaces_[10].set_value(goal_pose.orientation.x);  // pos_qx
    (void)command_interfaces_[11].set_value(goal_pose.orientation.y);  // pos_qy
    (void)command_interfaces_[12].set_value(goal_pose.orientation.z);  // pos_qz
    (void)command_interfaces_[13].set_value(goal_pose.orientation.w);  // pos_qw

    // Process via poses if available (only for circular motion)
    if (current_moprim_.type == MotionType::CIRCULAR_CARTESIAN && current_moprim_.poses.size() == 2)
    {
      const auto & via_pose = current_moprim_.poses[1].pose;            // via pose
      (void)command_interfaces_[14].set_value(via_pose.position.x);     // pos_via_x
      (void)command_interfaces_[15].set_value(via_pose.position.y);     // pos_via_y
      (void)command_interfaces_[16].set_value(via_pose.position.z);     // pos_via_z
      (void)command_interfaces_[17].set_value(via_pose.orientation.x);  // pos_via_qx
      (void)command_interfaces_[18].set_value(via_pose.orientation.y);  // pos_via_qy
      (void)command_interfaces_[19].set_value(via_pose.orientation.z);  // pos_via_qz
      (void)command_interfaces_[20].set_value(via_pose.orientation.w);  // pos_via_qw
    }
  }

  (void)command_interfaces_[21].set_value(current_moprim_.blend_radius);  // blend_radius

  // Read additional arguments
  for (const auto & arg : current_moprim_.additional_arguments)
  {
    if (arg.name == "velocity")
    {
      (void)command_interfaces_[22].set_value(arg.value);
    }
    else if (arg.name == "acceleration")
    {
      (void)command_interfaces_[23].set_value(arg.value);
    }
    else if (arg.name == "move_time")
    {
      (void)command_interfaces_[24].set_value(arg.value);
    }
    else
    {
      RCLCPP_WARN(get_node()->get_logger(), "Unknown additional argument: %s", arg.name.c_str());
    }
  }
  return true;
}

}  // namespace motion_primitives_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_controllers::MotionPrimitivesBaseController,
  controller_interface::ControllerInterface)
