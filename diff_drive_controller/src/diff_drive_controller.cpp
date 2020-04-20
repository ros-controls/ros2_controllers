// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "diff_drive_controller/diff_drive_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <utility>

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
}

namespace diff_drive_controller
{
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
using lifecycle_msgs::msg::State;

DiffDriveController::DiffDriveController()
: controller_interface::ControllerInterface() {}

DiffDriveController::DiffDriveController(
  std::vector<std::string> left_wheel_names,
  std::vector<std::string> right_wheel_names,
  std::vector<std::string> write_op_names)
: controller_interface::ControllerInterface(),
  left_wheel_names_(std::move(left_wheel_names)),
  right_wheel_names_(std::move(right_wheel_names)),
  write_op_names_(std::move(write_op_names))
{}

controller_interface::controller_interface_ret_t
DiffDriveController::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != CONTROLLER_INTERFACE_RET_SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "left_wheel_names",
    left_wheel_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "right_wheel_names",
    right_wheel_names_);
  lifecycle_node_->declare_parameter<double>("wheel_separation", wheel_params_.separation);
  lifecycle_node_->declare_parameter<int>("wheels_per_side", wheel_params_.wheels_per_side);
  lifecycle_node_->declare_parameter<double>("wheel_radius", wheel_params_.radius);
  lifecycle_node_->declare_parameter<double>(
    "wheel_separation_multiplier",
    wheel_params_.separation_multiplier);
  lifecycle_node_->declare_parameter<double>(
    "left_wheel_radius_multiplier",
    wheel_params_.left_radius_multiplier);
  lifecycle_node_->declare_parameter<double>(
    "right_wheel_radius_multiplier",
    wheel_params_.right_radius_multiplier);
  lifecycle_node_->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
  lifecycle_node_->declare_parameter<std::string>("base_frame_id", base_frame_id_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>("write_op_modes", write_op_names_);

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

controller_interface::controller_interface_ret_t DiffDriveController::update()
{
  auto logger = lifecycle_node_->get_logger();
  if (lifecycle_node_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  // Compute wheels velocities:
  const auto & current_command = *velocity_msg_ptr_;
  const auto & linear = current_command.linear.x;
  const auto & angular = current_command.angular.z;

  const double velocity_left = (linear - angular * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right = (linear + angular * wheel_separation / 2.0) / right_wheel_radius;

  // Set wheels velocities:
  for (size_t index = 0; index < wheels.wheels_per_side; ++index) {
    registered_left_wheel_handles_[index].command->set_cmd(velocity_left);
    registered_right_wheel_handles_[index].command->set_cmd(velocity_right);
  }

  set_op_mode(hardware_interface::OperationMode::ACTIVE);
  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = lifecycle_node_->get_logger();

  // update parameters
  left_wheel_names_ = lifecycle_node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = lifecycle_node_->get_parameter("right_wheel_names").as_string_array();
  wheel_params_.separation = lifecycle_node_->get_parameter("wheel_separation").as_double();
  wheel_params_.wheels_per_side =
    static_cast<size_t>(lifecycle_node_->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = lifecycle_node_->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    lifecycle_node_->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier = lifecycle_node_->get_parameter(
    "left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier = lifecycle_node_->get_parameter(
    "right_wheel_radius_multiplier").as_double();
  odom_frame_id_ = lifecycle_node_->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = lifecycle_node_->get_parameter("base_frame_id").as_string();

  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  const auto configure_side = [&logger](const std::string & side,
      const std::vector<std::string> & wheel_names,
      std::vector<WheelHandle> & registered_handles,
      const auto & robot_hardware) {
      if (wheel_names.empty()) {
        std::stringstream ss;
        ss << "No " << side << " wheel names specified.";
        RCLCPP_WARN(logger, ss.str().c_str());
      }

      // register handles
      registered_handles.resize(wheel_names.size());
      for (size_t index = 0; index < wheel_names.size(); ++index) {
        const auto wheel_name = wheel_names[index].c_str();
        auto & wheel_handle = registered_handles[index];

        auto result = robot_hardware->get_joint_state_handle(wheel_name, &wheel_handle.state);
        if (result != hardware_interface::HW_RET_OK) {
          RCLCPP_WARN(logger, "unable to obtain joint state handle for %s", wheel_name);
          return CallbackReturn::FAILURE;
        }

        auto ret = robot_hardware->get_joint_command_handle(wheel_name, &wheel_handle.command);
        if (ret != hardware_interface::HW_RET_OK) {
          RCLCPP_WARN(logger, "unable to obtain joint command handle for %s", wheel_name);
          return CallbackReturn::FAILURE;
        }
      }

      return CallbackReturn::SUCCESS;
    };

  if (auto robot_hardware = robot_hardware_.lock()) {
    const auto left_result =
      configure_side("left", left_wheel_names_, registered_left_wheel_handles_, robot_hardware);
    const auto right_result =
      configure_side("right", right_wheel_names_, registered_right_wheel_handles_, robot_hardware);

    if (left_result == CallbackReturn::FAILURE or right_result == CallbackReturn::FAILURE) {
      return CallbackReturn::FAILURE;
    }

    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index) {
      const auto op_name = write_op_names_[index].c_str();
      auto & op_handle = registered_operation_mode_handles_[index];

      auto result = robot_hardware->get_operation_mode_handle(op_name, &op_handle);
      if (result != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(logger, "unable to obtain operation mode handle for %s", op_name);
        return CallbackReturn::FAILURE;
      }
    }

  } else {
    return CallbackReturn::ERROR;
  }

  if (registered_left_wheel_handles_.empty() or registered_right_wheel_handles_.empty() or
    registered_operation_mode_handles_.empty())
  {
    return CallbackReturn::ERROR;
  }

  wheel_params_.wheels_per_side = registered_left_wheel_handles_.size();

  left_previous_commands_ = std::vector<double>(0, left_wheel_names_.size());
  right_previous_commands_ = std::vector<double>(0, right_wheel_names_.size());

  velocity_msg_ptr_ = std::make_shared<Twist>();

  auto callback = [this](const std::shared_ptr<Twist> msg) -> void {
      if (subscriber_is_active_) {
        *velocity_msg_ptr_ = *msg;
      }
    };

  velocity_command_subscriber_ =
    lifecycle_node_->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC,
    rclcpp::SystemDefaultsQoS(), callback);

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  is_halted = false;
  subscriber_is_active_ = true;
  //   traj_point_active_ptr_ = &traj_external_point_ptr_;
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  velocity_msg_ptr_ = std::make_shared<Twist>();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool DiffDriveController::reset()
{
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_operation_mode_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  velocity_msg_ptr_.reset();
  is_halted = false;
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DiffDriveController::set_op_mode(const hardware_interface::OperationMode & mode)
{
  for (auto & op_mode_handle : registered_operation_mode_handles_) {
    op_mode_handle->set_mode(mode);
  }
}

void DiffDriveController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles) {
      for (size_t index = 0; index < wheel_handles.size(); ++index) {
        const auto current_velocity = wheel_handles[index].state->get_velocity();
        const auto left_wheel_handle = wheel_handles[index];
        left_wheel_handle.command->set_cmd(current_velocity);
      }
    };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
  set_op_mode(hardware_interface::OperationMode::ACTIVE);
}
} // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_drive_controller::DiffDriveController,
  controller_interface::ControllerInterface)
