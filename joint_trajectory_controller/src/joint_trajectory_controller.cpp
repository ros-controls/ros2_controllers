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
using controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
using lifecycle_msgs::msg::State;

JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(),
  joint_names_({}),
  write_op_names_({})
{}

JointTrajectoryController::JointTrajectoryController(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & write_op_names)
: controller_interface::ControllerInterface(),
  joint_names_(joint_names),
  write_op_names_(write_op_names)
{}

controller_interface::controller_interface_ret_t
JointTrajectoryController::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != CONTROLLER_INTERFACE_RET_SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  lifecycle_node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>("write_op_modes", write_op_names_);

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

controller_interface::controller_interface_ret_t
JointTrajectoryController::update()
{
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
  auto traj_point_ptr = (*traj_point_active_ptr_)->sample(rclcpp::Clock().now());
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  auto logger = lifecycle_node_->get_logger();

  // update parameters
  joint_names_ = lifecycle_node_->get_parameter("joints").as_string_array();
  write_op_names_ = lifecycle_node_->get_parameter("write_op_modes").as_string_array();

  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (auto robot_hardware = robot_hardware_.lock()) {
    if (joint_names_.empty()) {
      RCLCPP_WARN(logger, "no joint names specified");
    }

    // register handles
    registered_joint_state_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_state_handle(
        joint_names_[index].c_str(), &registered_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain joint state handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_joint_cmd_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_command_handle(
        joint_names_[index].c_str(), &registered_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain joint command handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index) {
      auto ret = robot_hardware->get_operation_mode_handle(
        write_op_names_[index].c_str(), &registered_operation_mode_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain operation mode handle for %s", write_op_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (
    registered_joint_cmd_handles_.empty() ||
    registered_joint_state_handles_.empty() ||
    registered_operation_mode_handles_.empty())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
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
  auto callback = [this, &logger](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
    -> void
    {
      if (registered_joint_cmd_handles_.size() != msg->joint_names.size()) {
        RCLCPP_ERROR(
          logger,
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
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // TODO(karsten1987): no lifecyle for subscriber yet
  // joint_command_subscriber_->on_activate();

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  is_halted = false;
  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;

  // TODO(karsten1987): activate subscriptions of subscriber
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  subscriber_is_active_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  // TODO(karsten1987): what to do?

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ros_controllers::JointTrajectoryController, controller_interface::ControllerInterface)
