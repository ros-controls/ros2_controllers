///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, SRI International
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of SRI International nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian, Stu Glaser

#pragma once

namespace gripper_action_controller {

template <const char *HardwareInterface>
void GripperActionController<HardwareInterface>::preempt_active_goal() {
  // Cancels the currently active goal
  if (rt_active_goal_) {
    // Marks the current goal as canceled
    rt_active_goal_->setCanceled(
        std::make_shared<GripperCommandAction::Result>());
    rt_active_goal_.reset();
  }
}

template <const char *HardwareInterface>
controller_interface::return_type
GripperActionController<HardwareInterface>::init(
    const std::string &controller_name) {
  // initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  node_->declare_parameter<double>("action_monitor_rate", 20.0);
  node_->declare_parameter<std::string>("joint", joint_name_);
  node_->declare_parameter<double>("goal_tolerance", 0.01);
  node_->declare_parameter<double>("max_effort", 0.0);
  node_->declare_parameter<double>("stall_velocity_threshold", 0.001);
  node_->declare_parameter<double>("stall_timeout", 1.0);

  return controller_interface::return_type::SUCCESS;
}

template <const char *HardwareInterface>
controller_interface::return_type
GripperActionController<HardwareInterface>::update() {
  command_struct_rt_ = *(command_.readFromRT());

  const double current_position =
      joint_position_state_interface_->get_value();
  const double current_velocity =
      joint_velocity_state_interface_->get_value();

  const double error_position =
      command_struct_rt_.position_ - current_position;
  const double error_velocity = -current_velocity;

  check_for_success(node_->now(), error_position, current_position,
                    current_velocity);

  // Hardware interface adapter: Generate and send commands
  computed_command_ = hw_iface_adapter_.updateCommand(
      command_struct_rt_.position_, 0.0, error_position, error_velocity,
      command_struct_rt_.max_effort_);
  return controller_interface::return_type::SUCCESS;
}

template <const char *HardwareInterface>
rclcpp_action::GoalResponse
GripperActionController<HardwareInterface>::goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GripperCommandAction::Goal> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received & accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <const char *HardwareInterface>
void GripperActionController<HardwareInterface>::accepted_callback(
    std::shared_ptr<GoalHandle> goal_handle) { // Try to update goal
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

    last_movement_time_ = node_->now();
    rt_active_goal_ = rt_goal;
    rt_active_goal_->execute();
  }
  // Setup goal status checking timer
  goal_handle_timer_ = node_->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::seconds>(),
      std::bind(&RealtimeGoalHandle::runNonRealtime, rt_active_goal_));
}

template <const char *HardwareInterface>
rclcpp_action::CancelResponse
GripperActionController<HardwareInterface>::cancel_callback(
    const std::shared_ptr<GoalHandle> goal_handle) {
  RCLCPP_INFO(node_->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  if (rt_active_goal_ && rt_active_goal_->gh_ == goal_handle) {

    // Enter hold current position mode
    set_hold_position();

    RCLCPP_INFO(
        node_->get_logger(),
        "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<GripperCommandAction::Result>();
    rt_active_goal_->setCanceled(action_res);
    // Reset current goal
    rt_active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <const char *HardwareInterface>
void GripperActionController<HardwareInterface>::set_hold_position() {
  command_struct_.position_ = joint_position_state_interface_->get_value();
  command_struct_.max_effort_ = default_max_effort_;
  command_.writeFromNonRT(command_struct_);
}

template <const char *HardwareInterface>
void GripperActionController<HardwareInterface>::check_for_success(
    const rclcpp::Time &time, double error_position, double current_position,
    double current_velocity) {

  if (!rt_active_goal_)
    return;

  if (fabs(error_position) < goal_tolerance_) {
    pre_alloc_result_->effort = computed_command_;
    pre_alloc_result_->position = current_position;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    rt_active_goal_->setSucceeded(pre_alloc_result_);
    rt_active_goal_.reset();
  } else {
    if (fabs(current_velocity) > stall_velocity_threshold_) {
      last_movement_time_ = time;
    } else if ((time - last_movement_time_).seconds() > stall_timeout_) {
      pre_alloc_result_->effort = computed_command_;
      pre_alloc_result_->position = current_position;
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;
      rt_active_goal_->setAborted(pre_alloc_result_);
      rt_active_goal_.reset();
    }
  }
}

template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto logger = node_->get_logger();

  // Action status checking update rate
  const auto action_monitor_rate =
      node_->get_parameter("action_monitor_rate").as_double();
  action_monitor_period_ = rclcpp::Duration::from_seconds(
      1.0 / node_->get_parameter("action_monitor_rate").as_double());
  RCLCPP_INFO_STREAM(logger, "Action status changes will be monitored at "
      << action_monitor_rate << "Hz.");

  // Controlled joint
  joint_name_ = node_->get_parameter("joint").as_string();
  if (joint_name_.empty()) {
    RCLCPP_ERROR(logger, "Could not find joint name on param server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }

  // Default tolerances
  goal_tolerance_ = node_->get_parameter("goal_tolerance").as_double();
  goal_tolerance_ = fabs(goal_tolerance_);
  // Max allowable effort
  default_max_effort_ = node_->get_parameter("max_effort").as_double();
  default_max_effort_ = fabs(default_max_effort_);
  // Stall - stall velocity threshold, stall timeout
  stall_velocity_threshold_ =
      node_->get_parameter("stall_velocity_threshold").as_double();
  stall_timeout_ = node_->get_parameter("stall_timeout").as_double();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}
template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto position_command_interface_it =
      std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                   [](const hardware_interface::LoanedCommandInterface
                      &command_interface) {
                     return command_interface.get_interface_name() ==
                            hardware_interface::HW_IF_POSITION;
                   });
  if (position_command_interface_it == command_interfaces_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Expected 1 position command interface");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }
  if (position_command_interface_it->get_name() != joint_name_) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Position command interface is different than joint name `"
            << position_command_interface_it->get_name() << "` != `"
            << joint_name_ << "`");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }
  const auto position_state_interface_it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [](const hardware_interface::LoanedStateInterface &state_interface) {
        return state_interface.get_interface_name() ==
               hardware_interface::HW_IF_POSITION;
      });
  if (position_state_interface_it == state_interfaces_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Expected 1 position state interfaces");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }
  if (position_state_interface_it->get_name() != joint_name_) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Position state interface is different than joint name `"
            << position_state_interface_it->get_name() << "` != `"
            << joint_name_ << "`");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }
  const auto velocity_state_interface_it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [](const hardware_interface::LoanedStateInterface &state_interface) {
        return state_interface.get_interface_name() ==
               hardware_interface::HW_IF_VELOCITY;
      });
  if (velocity_state_interface_it == state_interfaces_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Expected 1 velocity state interfaces");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }
  if (velocity_state_interface_it->get_name() != joint_name_) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Velocity command interface is different than joint name `"
            << velocity_state_interface_it->get_name() << "` != `"
            << joint_name_ << "`");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  }

  joint_position_command_interface_ = &(*position_command_interface_it);
  joint_position_state_interface_ = &(*position_state_interface_it);
  joint_velocity_state_interface_ = &(*velocity_state_interface_it);

  // Hardware interface adapter
  hw_iface_adapter_.init(joint_position_command_interface_, node_);

  // Command - non RT version
  command_struct_.position_ = joint_position_state_interface_->get_value();
  command_struct_.max_effort_ = default_max_effort_;

  // Result
  pre_alloc_result_ =
      std::make_shared<control_msgs::action::GripperCommand::Result>();
  pre_alloc_result_->position = command_struct_.position_;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // Action interface
  using namespace std::placeholders;
  action_server_ =
      rclcpp_action::create_server<control_msgs::action::GripperCommand>(
          node_, "~/gripper_cmd",
          std::bind(&GripperActionController::goal_callback, this, _1, _2),
          std::bind(&GripperActionController::cancel_callback, this, _1),
          std::bind(&GripperActionController::accepted_callback, this, _1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}

template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  joint_position_command_interface_ = nullptr;
  joint_position_state_interface_ = nullptr;
  joint_velocity_state_interface_ = nullptr;
  release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}

template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}

template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_error(
    const rclcpp_lifecycle::State &previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}

template <const char *HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionController<HardwareInterface>::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn::SUCCESS;
}

template <const char *HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::command_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION}};
}

template <const char *HardwareInterface>
controller_interface::InterfaceConfiguration
GripperActionController<HardwareInterface>::state_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
           joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY}};
}

template <const char *HardwareInterface>
GripperActionController<HardwareInterface>::GripperActionController()
    : controller_interface::ControllerInterface(),
      action_monitor_period_(0){}

} // namespace gripper_action_controller
