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

#ifndef GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
#define GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_

#include "gripper_controllers/gripper_action_controller.hpp"

#include <memory>
#include <string>

namespace gripper_action_controller
{
template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::preempt_active_goal()
{
  // Cancels the currently active goal
  if (rt_active_goal_)
  {
    // Marks the current goal as canceled
    rt_active_goal_->setCanceled(std::make_shared<GripperCommandAction::Result>());
    rt_active_goal_.reset();
  }
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::return_type GripperActionController<HardwareInterface>::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  command_struct_rt_ = *(command_.readFromRT());

  const double current_position = joint_position_state_interface_->get().get_value();
  const double current_velocity = joint_velocity_state_interface_->get().get_value();

  const double error_position = command_struct_rt_.position_ - current_position;
  const double error_velocity = -current_velocity;

  check_for_success(get_node()->now(), error_position, current_position, current_velocity);

  // Hardware interface adapter: Generate and send commands
  computed_command_ = hw_iface_adapter_.updateCommand(
    command_struct_rt_.position_, 0.0, error_position, error_velocity,
    command_struct_rt_.max_effort_);
  return controller_interface::return_type::OK;
}

template <const char * HardwareInterface>
rclcpp_action::GoalResponse GripperActionController<HardwareInterface>::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommandAction::Goal>)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::accepted_callback(
  std::shared_ptr<GoalHandle> goal_handle)  // Try to update goal
{
  {
    auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

    // Accept new goal
    preempt_active_goal();

    // This is the non-realtime command_struct
    // We use command_ for sharing
    command_struct_.position_ = goal_handle->get_goal()->command.position;
    command_struct_.max_effort_ = goal_handle->get_goal()->command.max_effort;
    command_.writeFromNonRT(command_struct_);

    pre_alloc_result_->reached_goal = false;
    pre_alloc_result_->stalled = false;

    last_movement_time_ = get_node()->now();
    rt_active_goal_ = rt_goal;
    rt_active_goal_->execute();
  }
  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_active_goal_));
}

template <const char * HardwareInterface>
rclcpp_action::CancelResponse GripperActionController<HardwareInterface>::cancel_callback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  if (rt_active_goal_ && rt_active_goal_->gh_ == goal_handle)
  {
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<GripperCommandAction::Result>();
    rt_active_goal_->setCanceled(action_res);
    // Reset current goal
    rt_active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::set_hold_position()
{
  command_struct_.position_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  command_.writeFromNonRT(command_struct_);
}

template <const char * HardwareInterface>
void GripperActionController<HardwareInterface>::check_for_success(
  const rclcpp::Time & time, double error_position, double current_position,
  double current_velocity)
{
  if (!rt_active_goal_)
  {
    return;
  }

  if (fabs(error_position) < params_.goal_tolerance)
  {
    pre_alloc_result_->effort = computed_command_;
    pre_alloc_result_->position = current_position;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    RCLCPP_DEBUG(get_node()->get_logger(), "Successfully moved to goal.");
    rt_active_goal_->setSucceeded(pre_alloc_result_);
    rt_active_goal_.reset();
  }
  else
  {
    if (fabs(current_velocity) > params_.stall_velocity_threshold)
    {
      last_movement_time_ = time;
    }
    else if ((time - last_movement_time_).seconds() > params_.stall_timeout)
    {
      pre_alloc_result_->effort = computed_command_;
      pre_alloc_result_->position = current_position;
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;

      if (params_.allow_stalling)
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Returning success.");
        rt_active_goal_->setSucceeded(pre_alloc_result_);
      }
      else
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "Stall detected moving to goal. Aborting action!");
        rt_active_goal_->setAborted(pre_alloc_result_);
      }
      rt_active_goal_.reset();
    }
  }
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Action status checking update rate
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
  RCLCPP_INFO_STREAM(
    logger, "Action status changes will be monitored at " << params_.action_monitor_rate << "Hz.");

  // Controlled joint
  if (params_.joint.empty())
  {
    RCLCPP_ERROR(logger, "Joint name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto position_command_interface_it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [](const hardware_interface::LoanedCommandInterface & command_interface)
    { return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
  if (position_command_interface_it == command_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 position command interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (position_command_interface_it->get_prefix_name() != params_.joint)
  {
    RCLCPP_ERROR_STREAM(
      get_node()->get_logger(), "Position command interface is different than joint name `"
                                  << position_command_interface_it->get_prefix_name() << "` != `"
                                  << params_.joint << "`");
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
    RCLCPP_ERROR_STREAM(
      get_node()->get_logger(), "Position state interface is different than joint name `"
                                  << position_state_interface_it->get_prefix_name() << "` != `"
                                  << params_.joint << "`");
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
    RCLCPP_ERROR_STREAM(
      get_node()->get_logger(), "Velocity command interface is different than joint name `"
                                  << velocity_state_interface_it->get_prefix_name() << "` != `"
                                  << params_.joint << "`");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_position_command_interface_ = *position_command_interface_it;
  joint_position_state_interface_ = *position_state_interface_it;
  joint_velocity_state_interface_ = *velocity_state_interface_it;

  // Hardware interface adapter
  hw_iface_adapter_.init(joint_position_command_interface_, get_node());

  // Command - non RT version
  command_struct_.position_ = joint_position_state_interface_->get().get_value();
  command_struct_.max_effort_ = params_.max_effort;
  command_.initRT(command_struct_);

  // Result
  pre_alloc_result_ = std::make_shared<control_msgs::action::GripperCommand::Result>();
  pre_alloc_result_->position = command_struct_.position_;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // Action interface
  action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
    get_node(), "~/gripper_cmd",
    std::bind(
      &GripperActionController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperActionController::cancel_callback, this, std::placeholders::_1),
    std::bind(&GripperActionController::accepted_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::CallbackReturn GripperActionController<HardwareInterface>::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  joint_position_command_interface_ = std::nullopt;
  joint_position_state_interface_ = std::nullopt;
  joint_velocity_state_interface_ = std::nullopt;
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

template <const char * HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + hardware_interface::HW_IF_POSITION}};
}

template <const char * HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + hardware_interface::HW_IF_POSITION,
     params_.joint + "/" + hardware_interface::HW_IF_VELOCITY}};
}

template <const char * HardwareInterface>
GripperActionController<HardwareInterface>::GripperActionController()
: controller_interface::ControllerInterface(),
  action_monitor_period_(rclcpp::Duration::from_seconds(0))
{
}

}  // namespace gripper_action_controller

#endif  // GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_IMPL_HPP_
