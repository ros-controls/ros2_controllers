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

#include "ros_controllers/joint_trajectory_controller.hpp"

#include <cassert>
#include <chrono>
#include <iterator>
#include <string>
#include <memory>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ros_controllers
{

using namespace std::chrono_literals;

namespace
{

rcl_lifecycle_transition_key_t
fetch_parameters_from_parameter_server(
  std::shared_ptr<rclcpp::parameter_client::AsyncParametersClient> parameters_client,
  const std::string parameter_key,
  std::vector<std::string> & parameters)
{
  auto list_future = parameters_client->list_parameters({parameter_key}, 0);
  std::future_status status;
  do {
    status = list_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::timeout) {
      auto error_msg = std::string("couldn't fetch parameters for key: ") + parameter_key;
      RCUTILS_LOG_ERROR(error_msg.c_str())
      return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
    }
  } while (status != std::future_status::ready);
  auto parameter_names = list_future.get();

  if (parameter_names.names.size() == 0) {
    auto error_msg = std::string("no results found for key: ") + parameter_key;
    RCUTILS_LOG_ERROR(error_msg.c_str())
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }

  auto get_future = parameters_client->get_parameters({parameter_names.names});
  do {
    status = get_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::timeout) {
      auto error_msg = std::string("couldn't get parameters for :");
      for (auto & name : parameter_names.names) {
        error_msg += " " + name;
      }
      RCUTILS_LOG_ERROR(error_msg.c_str())
      return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
    }
  } while (status != std::future_status::ready);
  auto parameter_values = get_future.get();

  if (parameter_values.size() == 0) {
    auto error_msg = std::string("couldn't get parameters for :");
    for (auto & name : parameter_names.names) {
      error_msg += " " + name;
    }
    RCUTILS_LOG_ERROR(error_msg.c_str())
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  for (auto pv : parameter_values) {
    parameters.push_back(pv.as_string());
  }

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

}  // namespace

JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface()
{}

JointTrajectoryController::JointTrajectoryController(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & write_op_names)
: controller_interface::ControllerInterface(),
  joint_names_(joint_names),
  write_op_names_(write_op_names)
{}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  auto max_wait = 2u;
  auto wait = 0u;
  while (!parameters_client_->wait_for_service(1s) && (wait++) < max_wait) {
    if (!rclcpp::ok()) {
      RCUTILS_LOG_ERROR("waiting for parameter server got interrupted")
      return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
    }
  }
  if (wait <= max_wait) {
    std::string joint_parameter_key =
      std::string(".") + lifecycle_node_->get_name() + ".joints";
    auto ret = fetch_parameters_from_parameter_server(
      parameters_client_, joint_parameter_key, joint_names_);
    if (ret != lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
      return ret;
    }

    std::string write_op_mode_key =
      std::string(".") + lifecycle_node_->get_name() + ".write_op_modes";
    ret = fetch_parameters_from_parameter_server(
      parameters_client_, write_op_mode_key, write_op_names_);
    if (ret != lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
      return ret;
    }
  } else {
    RCUTILS_LOG_INFO("parameter server not available")
  }

  if (!reset()) {
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
  }

  if (auto robot_hardware = robot_hardware_.lock()) {
    // register handles
    registered_joint_state_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_state_handle(
        joint_names_[index].c_str(), &registered_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
      }
    }
    registered_joint_cmd_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_command_handle(
        joint_names_[index].c_str(), &registered_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
      }
    }
    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index) {
      auto ret = robot_hardware->get_operation_mode_handle(
        write_op_names_[index].c_str(), &registered_operation_mode_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
      }
    }
  } else {
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
  }

  if (
    registered_joint_cmd_handles_.empty() ||
    registered_joint_state_handles_.empty() ||
    registered_operation_mode_handles_.empty())
  {
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(registered_joint_state_handles_.size());
  for (size_t index = 0; index < registered_joint_state_handles_.size(); ++index) {
    traj_msg_home_ptr_->points[0].positions[index] =
      registered_joint_state_handles_[index]->get_position();
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_home_point_ptr_ = std::make_shared<Trajectory>();

  // subscriber call back
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this](const typename trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    -> void
    {
      if (registered_joint_cmd_handles_.size() != msg->joint_names.size()) {
        RCUTILS_LOG_FATAL_NAMED(
          "joint command subscriber",
          "number of joints in joint trajectory msg (%d) "
          "does not match number of joint command handles (%d)\n",
          msg->joint_names.size(), registered_joint_cmd_handles_.size());
      }

      // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
      // always replace old msg with new one for now
      if (subscriber_is_active_) {
        traj_external_point_ptr_->update(msg);
      }
    };

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ =
    lifecycle_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", callback);

  // TODO(karsten1987): no lifecyle for subscriber yet
  // joint_command_subscriber_->on_activate();

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  is_halted = false;
  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;

  // TODO(karsten1987): activate subscriptions of subscriber
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  subscriber_is_active_ = false;

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  if (!reset()) {
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR;
  }
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

bool
JointTrajectoryController::reset()
{
  // TODO(karsten1987): need a way to re-fetch names after reset. Uncomment this in the future
  // joint_names_.clear();
  // write_op_names_.clear();

  registered_joint_cmd_handles_.clear();
  registered_joint_state_handles_.clear();
  registered_operation_mode_handles_.clear();

  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();

  // iterator has no default value
  // prev_traj_point_ptr_;
  traj_point_active_ptr_ = nullptr;
  traj_external_point_ptr_.reset();
  traj_home_point_ptr_.reset();
  traj_msg_home_ptr_.reset();

  is_halted = false;

  return true;
}

rcl_lifecycle_transition_key_t
JointTrajectoryController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  // TODO(karsten1987): what to do?

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

controller_interface::controller_interface_ret_t
JointTrajectoryController::update()
{
  using controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  using lifecycle_msgs::msg::State;

  if (lifecycle_node_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  // when no traj msg has been received yet
  if (!traj_point_active_ptr_ || (*traj_point_active_ptr_)->is_empty()) {
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  // find next new point for current timestamp
  auto traj_point_ptr = (*traj_point_active_ptr_)->sample(rclcpp::Time::now());
  // find next new point for current timestamp
  // set cmd only if a point is found
  if (traj_point_ptr == (*traj_point_active_ptr_)->end()) {
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  // check if new point ptr points to the same as previous point
  if (prev_traj_point_ptr_ == traj_point_ptr) {
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  size_t joint_num = registered_joint_cmd_handles_.size();
  for (size_t index = 0; index < joint_num; ++index) {
    registered_joint_cmd_handles_[index]->set_cmd(traj_point_ptr->positions[index]);
  }

  prev_traj_point_ptr_ = traj_point_ptr;
  set_op_mode(hardware_interface::OperationMode::ACTIVE);

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

void
JointTrajectoryController::set_op_mode(const hardware_interface::OperationMode & mode)
{
  for (auto & op_mode_handle : registered_operation_mode_handles_) {
    op_mode_handle->set_mode(mode);
  }
}

void
JointTrajectoryController::halt()
{
  size_t joint_num = registered_joint_cmd_handles_.size();
  for (size_t index = 0; index < joint_num; ++index) {
    registered_joint_cmd_handles_[index]->set_cmd(
      registered_joint_state_handles_[index]->get_position());
  }
  set_op_mode(hardware_interface::OperationMode::ACTIVE);
}

}  // namespace ros_controllers

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(
  ros_controllers::JointTrajectoryController, controller_interface::ControllerInterface)
