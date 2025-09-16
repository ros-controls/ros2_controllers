// Copyright 2014, SRI International
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

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian, Stu Glaser

#ifndef PARALLEL_GRIPPER_CONTROLLER__PARALLEL_GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
#define PARALLEL_GRIPPER_CONTROLLER__PARALLEL_GRIPPER_ACTION_CONTROLLER_IMPL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "parallel_gripper_controller/parallel_gripper_action_controller.hpp"

namespace parallel_gripper_action_controller
{

void GripperActionController::preempt_active_goal()
{
  // Cancels the currently active goal
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    // Marks the current goal as canceled
    active_goal->setCanceled(std::make_shared<GripperCommandAction::Result>());
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

controller_interface::CallbackReturn GripperActionController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperActionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  command_struct_rt_ = *(command_.readFromRT());

  const double current_position = joint_position_state_interface_->get().get_value();
  const double current_velocity = joint_velocity_state_interface_->get().get_value();
  const double error_position = command_struct_rt_.position_cmd_ - current_position;

  check_for_success(get_node()->now(), error_position, current_position, current_velocity);

  joint_command_interface_->get().set_value(command_struct_rt_.position_cmd_);
  if (speed_interface_.has_value())
  {
    speed_interface_->get().set_value(command_struct_rt_.max_velocity_);
  }
  if (effort_interface_.has_value())
  {
    effort_interface_->get().set_value(command_struct_rt_.max_effort_);
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse GripperActionController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommandAction::Goal> goal_handle)
{
  if (goal_handle->command.position.size() != 1)
  {
    pre_alloc_result_ = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
    pre_alloc_result_->state.position.resize(1);
    pre_alloc_result_->state.effort.resize(1);
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received action goal with wrong number of position values, expects 1, got %zu",
      goal_handle->command.position.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperActionController::accepted_callback(
  std::shared_ptr<GoalHandle> goal_handle)  // Try to update goal
{
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  // Accept new goal
  preempt_active_goal();

  // This is the non-realtime command_struct
  // We use command_ for sharing
  command_struct_.position_cmd_ = goal_handle->get_goal()->command.position[0];
  if (!params_.max_velocity_interface.empty() && !goal_handle->get_goal()->command.velocity.empty())
  {
    command_struct_.max_velocity_ = goal_handle->get_goal()->command.velocity[0];
  }
  else
  {
    command_struct_.max_velocity_ = params_.max_velocity;
  }
  if (!params_.max_effort_interface.empty() && !goal_handle->get_goal()->command.effort.empty())
  {
    command_struct_.max_effort_ = goal_handle->get_goal()->command.effort[0];
  }
  else
  {
    command_struct_.max_effort_ = params_.max_effort;
  }
  command_.writeFromNonRT(command_struct_);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  last_movement_time_ = get_node()->now();
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

rclcpp_action::CancelResponse GripperActionController::cancel_callback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<GripperCommandAction::Result>();
    active_goal->setCanceled(action_res);
    // Reset current goal
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionController::set_hold_position()
{
  command_struct_.position_cmd_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  command_struct_.max_velocity_ = params_.max_velocity;
  command_.writeFromNonRT(command_struct_);
}

void GripperActionController::check_for_success(
  const rclcpp::Time & time, double error_position, double current_position,
  double current_velocity)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (!active_goal)
  {
    return;
  }

  if (fabs(error_position) < params_.goal_tolerance)
  {
    pre_alloc_result_->state.effort[0] = computed_command_;
    pre_alloc_result_->state.position[0] = current_position;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    RCLCPP_DEBUG(get_node()->get_logger(), "Successfully moved to goal.");
    active_goal->setSucceeded(pre_alloc_result_);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  else
  {
    if (fabs(current_velocity) > params_.stall_velocity_threshold)
    {
      last_movement_time_ = time;
    }
    else if ((time - last_movement_time_).seconds() > params_.stall_timeout)
    {
      pre_alloc_result_->state.effort[0] = computed_command_;
      pre_alloc_result_->state.position[0] = current_position;
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;

      if (params_.allow_stalling)
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Returning success.");
        active_goal->setSucceeded(pre_alloc_result_);
      }
      else
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Aborting action!");
        active_goal->setAborted(pre_alloc_result_);
      }
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }
}

controller_interface::CallbackReturn GripperActionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  params_ = param_listener_->get_params();

  // Action status checking update rate
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %f Hz.", params_.action_monitor_rate);

  // Controlled joint
  if (params_.joint.empty())
  {
    RCLCPP_ERROR(logger, "Joint name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger, "Joint name is : %s", params_.joint.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperActionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto command_interface_it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [](const hardware_interface::LoanedCommandInterface & command_interface)
    { return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
  if (command_interface_it == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected 1 %s command interface",
      hardware_interface::HW_IF_POSITION);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Command interface is different than joint name `%s` != `%s`",
      command_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto position_state_interface_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [](const hardware_interface::LoanedStateInterface & state_interface)
    { return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
  if (position_state_interface_it == state_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 position state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (position_state_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Position state interface is different than joint name `%s` != `%s`",
      position_state_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto velocity_state_interface_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [](const hardware_interface::LoanedStateInterface & state_interface)
    { return state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY; });
  if (velocity_state_interface_it == state_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 velocity state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (velocity_state_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Velocity command interface is different than joint name `%s` != `%s`",
      velocity_state_interface_it->get_prefix_name().c_str(), params_.joint.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_command_interface_ = *command_interface_it;
  joint_position_state_interface_ = *position_state_interface_it;
  joint_velocity_state_interface_ = *velocity_state_interface_it;

  for (auto & interface : command_interfaces_)
  {
    if (interface.get_interface_name() == "set_gripper_max_effort")
    {
      effort_interface_ = interface;
    }
    else if (interface.get_interface_name() == "set_gripper_max_velocity")
    {
      speed_interface_ = interface;
    }
  }

  // Command - non RT version
  command_struct_.position_cmd_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  command_struct_.max_velocity_ = params_.max_velocity;
  command_.initRT(command_struct_);

  // Result
  pre_alloc_result_ = std::make_shared<control_msgs::action::ParallelGripperCommand::Result>();
  pre_alloc_result_->state.position.resize(1);
  pre_alloc_result_->state.effort.resize(1);
  pre_alloc_result_->state.position[0] = command_struct_.position_cmd_;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // Action interface
  action_server_ = rclcpp_action::create_server<control_msgs::action::ParallelGripperCommand>(
    get_node(), "~/gripper_cmd",
    std::bind(
      &GripperActionController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperActionController::cancel_callback, this, std::placeholders::_1),
    std::bind(&GripperActionController::accepted_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperActionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  joint_command_interface_ = std::nullopt;
  joint_position_state_interface_ = std::nullopt;
  joint_velocity_state_interface_ = std::nullopt;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GripperActionController::command_interface_configuration() const
{
  std::vector<std::string> names = {params_.joint + "/" + hardware_interface::HW_IF_POSITION};
  if (!params_.max_effort_interface.empty())
  {
    names.push_back({params_.max_effort_interface});
  }
  if (!params_.max_velocity_interface.empty())
  {
    names.push_back({params_.max_velocity_interface});
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
GripperActionController::state_interface_configuration() const
{
  std::vector<std::string> interface_names;
  for (const auto & interface : params_.state_interfaces)
  {
    interface_names.push_back(params_.joint + "/" + interface);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
}

GripperActionController::GripperActionController()
: controller_interface::ControllerInterface(),
  action_monitor_period_(rclcpp::Duration::from_seconds(0))
{
}

}  // namespace parallel_gripper_action_controller

#endif  // PARALLEL_GRIPPER_CONTROLLER__PARALLEL_GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
