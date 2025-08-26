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

#include "motion_primitives_forward_controller/motion_primitives_forward_controller.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/helpers.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace motion_primitives_forward_controller
{
MotionPrimitivesForwardController::MotionPrimitivesForwardController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_init()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Initializing Motion Primitives Forward Controller");
  try
  {
    param_listener_ =
      std::make_shared<motion_primitives_forward_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Configuring Motion Primitives Forward Controller");

  params_ = param_listener_->get_params();
  tf_prefix_ = params_.tf_prefix;

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<ExecuteMotionAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/motion_sequence",
    std::bind(&MotionPrimitivesForwardController::goal_received_callback, this, _1, _2),
    std::bind(&MotionPrimitivesForwardController::goal_cancelled_callback, this, _1),
    std::bind(&MotionPrimitivesForwardController::goal_accepted_callback, this, _1));

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesForwardController::command_interface_configuration() const
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
MotionPrimitivesForwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(2);
  state_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/execution_status");
  state_interfaces_config.names.push_back(tf_prefix_ + "motion_primitive/ready_for_new_primitive");
  return state_interfaces_config;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionPrimitivesForwardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (cancel_requested_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Cancel requested, stopping execution.");
    cancel_requested_ = false;
    reset_command_interfaces();
    // send stop command immediately to the hw-interface
    (void)command_interfaces_[0].set_value(static_cast<double>(MotionHelperType::STOP_MOTION));
    // clear the queue (ignore return value)
    static_cast<void>(moprim_queue_.get_latest(current_moprim_));
    robot_stop_requested_ = true;
  }

  // read the status from the state interface
  auto opt_value_execution = state_interfaces_[0].get_optional();
  if (!opt_value_execution.has_value())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "State interface 0 (execution_state) returned no value.");
    return controller_interface::return_type::ERROR;
  }
  execution_status_ =
    static_cast<ExecutionState>(static_cast<uint8_t>(std::round(opt_value_execution.value())));
  switch (execution_status_)
  {
    case ExecutionState::IDLE:
      print_error_once_ = true;
      was_executing_ = false;
      break;
    case ExecutionState::EXECUTING:
      if (!was_executing_)
      {
        was_executing_ = true;
      }
      print_error_once_ = true;
      break;

    case ExecutionState::SUCCESS:
      print_error_once_ = true;
      if (has_active_goal_ && was_executing_)
      {
        rt_goal_handle_.try_get(
          [&](const std::shared_ptr<RealtimeGoalHandle> & goal_handle)
          {
            was_executing_ = false;
            auto result = std::make_shared<ExecuteMotionAction::Result>();
            result->error_code = ExecuteMotionAction::Result::SUCCESSFUL;
            result->error_string = "Motion primitives executed successfully";
            goal_handle->setSucceeded(result);
            has_active_goal_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "Motion primitives executed successfully.");
          });
      }

      break;

    case ExecutionState::STOPPING:
      RCLCPP_DEBUG(get_node()->get_logger(), "Execution state: STOPPING");
      print_error_once_ = true;
      was_executing_ = false;
      break;

    case ExecutionState::STOPPED:
      print_error_once_ = true;
      was_executing_ = false;
      if (has_active_goal_)
      {
        rt_goal_handle_.try_get(
          [&](const std::shared_ptr<RealtimeGoalHandle> & goal_handle)
          {
            auto result = std::make_shared<ExecuteMotionAction::Result>();
            goal_handle->setCanceled(result);
            has_active_goal_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "Motion primitives execution canceled.");
          });
      }
      if (robot_stop_requested_)
      {
        // If the robot was stopped by a stop command, reset the command interfaces
        // to allow new motion primitives to be sent.
        reset_command_interfaces();
        (void)command_interfaces_[0].set_value(static_cast<double>(MotionHelperType::RESET_STOP));
        robot_stop_requested_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "Robot stopped, ready for new motion primitives.");
      }

      break;

    case ExecutionState::ERROR:
      was_executing_ = false;
      if (print_error_once_)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Execution state: ERROR");
        print_error_once_ = false;
      }
      break;

    default:
      RCLCPP_ERROR(
        get_node()->get_logger(), "Error: Unknown execution status: %d",
        static_cast<uint8_t>(execution_status_));
      return controller_interface::return_type::ERROR;
  }

  auto opt_value_ready = state_interfaces_[1].get_optional();
  if (!opt_value_ready.has_value())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "State interface 1 (ready_for_new_primitive) returned no value.");
    return controller_interface::return_type::ERROR;
  }
  ready_for_new_primitive_ =
    static_cast<ReadyForNewPrimitive>(static_cast<uint8_t>(std::round(opt_value_ready.value())));

  if (!cancel_requested_)
  {
    switch (ready_for_new_primitive_)
    {
      case ReadyForNewPrimitive::NOT_READY:
      {
        return controller_interface::return_type::OK;
      }
      case ReadyForNewPrimitive::READY:
      {
        if (moprim_queue_.empty())  // check if new command is available
        {
          return controller_interface::return_type::OK;
        }
        else
        {
          if (!set_command_interfaces())
          {
            RCLCPP_ERROR(get_node()->get_logger(), "Error: set_command_interfaces() failed");
            return controller_interface::return_type::ERROR;
          }
          return controller_interface::return_type::OK;
        }
      }
      default:
        RCLCPP_ERROR(
          get_node()->get_logger(), "Error: Unknown state for ready_for_new_primitive: %d",
          static_cast<uint8_t>(ready_for_new_primitive_));
        return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

// Reset Command-Interfaces to nan
void MotionPrimitivesForwardController::reset_command_interfaces()
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
bool MotionPrimitivesForwardController::set_command_interfaces()
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

rclcpp_action::GoalResponse MotionPrimitivesForwardController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteMotionAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new trajectories. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (has_active_goal_)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Already has an active goal, rejecting new goal request.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto & primitives = goal->trajectory.motions;

  if (robot_stop_requested_)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Robot requested to stop. Discarding the new command.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (primitives.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: no motion primitives provided.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (
    (primitives.size() == 1) &&
    primitives.size() > (moprim_queue_.capacity() - moprim_queue_.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Remaining queue capacity (%zu) is not enough for the requested %zu motion primitive.",
      moprim_queue_.capacity() - moprim_queue_.size(), primitives.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  else if ((primitives.size() + 2) > (moprim_queue_.capacity() - moprim_queue_.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Remaining queue capacity (%zu) is not enough for the requested %zu motion primitives (+ "
      "start and stop marker).",
      moprim_queue_.capacity() - moprim_queue_.size(), primitives.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (size_t i = 0; i < primitives.size(); ++i)
  {
    const auto & primitive = primitives[i];

    switch (static_cast<uint8_t>(primitive.type))
    {
      case MotionType::LINEAR_JOINT:
        RCLCPP_INFO(get_node()->get_logger(), "Primitive %zu: LINEAR_JOINT (PTP)", i);
        if (primitive.joint_positions.empty())
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Primitive %zu invalid: LINEAR_JOINT requires joint_positions.", i);
          return rclcpp_action::GoalResponse::REJECT;
        }
        break;

      case MotionType::LINEAR_CARTESIAN:
        RCLCPP_INFO(get_node()->get_logger(), "Primitive %zu: LINEAR_CARTESIAN (LIN)", i);
        if (primitive.poses.empty())
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Primitive %zu invalid: LINEAR_CARTESIAN requires at least one pose.", i);
          return rclcpp_action::GoalResponse::REJECT;
        }
        break;

      case MotionType::CIRCULAR_CARTESIAN:
        RCLCPP_INFO(get_node()->get_logger(), "Primitive %zu: CIRCULAR_CARTESIAN (CIRC)", i);
        if (primitive.poses.size() != 2)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Primitive %zu invalid: CIRCULAR_CARTESIAN requires exactly two poses.", i);
          return rclcpp_action::GoalResponse::REJECT;
        }
        break;

      default:
        RCLCPP_ERROR(
          get_node()->get_logger(), "Primitive %zu invalid: unknown motion type %u.", i,
          primitive.type);
        return rclcpp_action::GoalResponse::REJECT;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPrimitivesForwardController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotionAction>>)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPrimitivesForwardController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotionAction>> goal_handle)
{
  const auto & primitives = goal_handle->get_goal()->trajectory.motions;

  auto add_motions = [this](const std::vector<MotionPrimitive> & motion_primitives)
  {
    for (const auto & primitive : motion_primitives)
    {
      if (!moprim_queue_.push(primitive))
      {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to push motion primitive to queue.");
      }
    }
  };

  if (primitives.size() > 1)
  {
    MotionPrimitive start_marker;
    start_marker.type = static_cast<uint8_t>(MotionHelperType::MOTION_SEQUENCE_START);
    if (!moprim_queue_.push(start_marker))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Failed to push motion sequence start marker to queue.");
    }

    add_motions(primitives);

    MotionPrimitive end_marker;
    end_marker.type = static_cast<uint8_t>(MotionHelperType::MOTION_SEQUENCE_END);
    if (!moprim_queue_.push(end_marker))
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to push motion sequence end marker to queue.");
    }
  }
  else
  {
    add_motions(primitives);
  }

  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal_handle_.set(
    [&](auto & handle)
    {
      handle = rt_goal;
      has_active_goal_ = true;
    });

  rt_goal->execute();

  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));

  RCLCPP_INFO(
    get_node()->get_logger(), "Accepted goal with %zu motion primitives.", primitives.size());
}

}  // namespace motion_primitives_forward_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_forward_controller::MotionPrimitivesForwardController,
  controller_interface::ControllerInterface)
