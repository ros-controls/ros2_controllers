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
#include "motion_primitives_forward_controller/execution_state.hpp"
#include "motion_primitives_forward_controller/motion_type.hpp"
#include "motion_primitives_forward_controller/ready_for_new_primitive.hpp"

namespace motion_primitives_forward_controller
{
MotionPrimitivesForwardController::MotionPrimitivesForwardController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Initializing Motion Primitives Forward Controller");

  try
  {
    param_listener_ =
      std::make_shared<motion_primitives_forward_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring Motion Primitives Forward Controller");

  params_ = param_listener_->get_params();

  // Check if the name is not empty
  if (params_.name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error: A name must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check if there are exactly 25 command interfaces
  if (params_.command_interfaces.size() != 25)
  {  // motion_type + 6 joints + 2*7 positions + blend_radius + velocity + acceleration + move_time
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error: Exactly 25 command interfaces must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check if there are exactly 2 state interfaces
  if (params_.state_interfaces.size() != 2)
  {  // execution_status + ready_for_new_primitive
    RCLCPP_ERROR(get_node()->get_logger(), "Error: Exactly two state interfaces must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  action_server_ = rclcpp_action::create_server<ExecuteMotion>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/motion_sequence",
    std::bind(
      &MotionPrimitivesForwardController::goal_received_callback, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &MotionPrimitivesForwardController::goal_cancelled_callback, this, std::placeholders::_1),
    std::bind(
      &MotionPrimitivesForwardController::goal_accepted_callback, this, std::placeholders::_1));

  queue_size_ = params_.queue_size;
  if (queue_size_ == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error: Queue size must be greater than 0!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesForwardController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.command_interfaces.size());

  // Iterate over all command interfaces from the config yaml file
  for (const auto & interface_name : params_.command_interfaces)
  {
    command_interfaces_config.names.push_back(params_.name + "/" + interface_name);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesForwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params_.state_interfaces.size());

  // Iterate over all state interfaces from the config yaml file
  for (const auto & interface_name : params_.state_interfaces)
  {
    state_interfaces_config.names.push_back(params_.name + "/" + interface_name);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionPrimitivesForwardController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // read the status from the state interface
  auto opt_value_execution = state_interfaces_[0].get_optional();
  if (!opt_value_execution.has_value())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "State interface 0 (execution_state) returned no value.");
    return controller_interface::return_type::ERROR;
  }
  execution_status_ = static_cast<int8_t>(std::round(opt_value_execution.value()));

  switch (execution_status_)
  {
    case ExecutionState::IDLE:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: IDLE");
      print_error_once_ = true;
      break;
    case ExecutionState::EXECUTING:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: EXECUTING");
      if (!was_executing_)
      {
        was_executing_ = true;
      }
      print_error_once_ = true;
      break;

    case ExecutionState::SUCCESS:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: SUCCESS");
      print_error_once_ = true;
      break;

    case ExecutionState::STOPPED:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: STOPPED");
      print_error_once_ = true;
      break;

    case ExecutionState::ERROR:
      if (print_error_once_)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Execution state: ERROR");
        print_error_once_ = false;
      }
      break;

    default:
      RCLCPP_ERROR(
        get_node()->get_logger(), "Error: Unknown execution status: %d", execution_status_);
      return controller_interface::return_type::ERROR;
  }

  auto opt_value_ready = state_interfaces_[1].get_optional();
  if (!opt_value_ready.has_value())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "State interface 1 (ready_for_new_primitive) returned no value.");
    return controller_interface::return_type::ERROR;
  }
  uint8_t ready_for_new_primitive = static_cast<int8_t>(std::round(opt_value_ready.value()));

  if (!moprim_queue_.empty())  // check if new command is available
  {
    switch (ready_for_new_primitive)
    {
      case ReadyForNewPrimitive::NOT_READY:
      {
        return controller_interface::return_type::OK;
      }
      case ReadyForNewPrimitive::READY:
      {
        if (!set_command_interfaces())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Error: set_command_interfaces() failed");
          return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
      }
      default:
        RCLCPP_ERROR(
          get_node()->get_logger(), "Error: Unknown state for ready_for_new_primitive: %d",
          ready_for_new_primitive);
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
  std::lock_guard<std::mutex> guard(command_mutex_);
  // Get the oldest message from the queue
  std::shared_ptr<MotionPrimitiveMsg> current_moprim = moprim_queue_.front();
  moprim_queue_.pop();

  // Check if the message is valid
  if (!current_moprim)
  {
    RCLCPP_WARN(get_node()->get_logger(), "No valid reference message received");
    return false;
  }

  // Set the motion_type
  (void)command_interfaces_[0].set_value(static_cast<double>(current_moprim->type));

  // Process joint positions if available
  if (!current_moprim->joint_positions.empty())
  {
    for (size_t i = 0; i < current_moprim->joint_positions.size(); ++i)
    {
      (void)command_interfaces_[i + 1].set_value(current_moprim->joint_positions[i]);  // q1 to q6
    }
  }

  // Process Cartesian poses if available
  if (!current_moprim->poses.empty())
  {
    const auto & goal_pose = current_moprim->poses[0].pose;            // goal pose
    (void)command_interfaces_[7].set_value(goal_pose.position.x);      // pos_x
    (void)command_interfaces_[8].set_value(goal_pose.position.y);      // pos_y
    (void)command_interfaces_[9].set_value(goal_pose.position.z);      // pos_z
    (void)command_interfaces_[10].set_value(goal_pose.orientation.x);  // pos_qx
    (void)command_interfaces_[11].set_value(goal_pose.orientation.y);  // pos_qy
    (void)command_interfaces_[12].set_value(goal_pose.orientation.z);  // pos_qz
    (void)command_interfaces_[13].set_value(goal_pose.orientation.w);  // pos_qw

    // Process via poses if available (only for circular motion)
    if (current_moprim->type == MotionType::CIRCULAR_CARTESIAN && current_moprim->poses.size() == 2)
    {
      const auto & via_pose = current_moprim->poses[1].pose;            // via pose
      (void)command_interfaces_[14].set_value(via_pose.position.x);     // pos_via_x
      (void)command_interfaces_[15].set_value(via_pose.position.y);     // pos_via_y
      (void)command_interfaces_[16].set_value(via_pose.position.z);     // pos_via_z
      (void)command_interfaces_[17].set_value(via_pose.orientation.x);  // pos_via_qx
      (void)command_interfaces_[18].set_value(via_pose.orientation.y);  // pos_via_qy
      (void)command_interfaces_[19].set_value(via_pose.orientation.z);  // pos_via_qz
      (void)command_interfaces_[20].set_value(via_pose.orientation.w);  // pos_via_qw
    }
  }

  (void)command_interfaces_[21].set_value(current_moprim->blend_radius);  // blend_radius

  // Read additional arguments
  for (const auto & arg : current_moprim->additional_arguments)
  {
    if (arg.argument_name == "velocity")
    {
      (void)command_interfaces_[22].set_value(arg.argument_value);
    }
    else if (arg.argument_name == "acceleration")
    {
      (void)command_interfaces_[23].set_value(arg.argument_value);
    }
    else if (arg.argument_name == "move_time")
    {
      (void)command_interfaces_[24].set_value(arg.argument_value);
    }
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Unknown additional argument: %s", arg.argument_name.c_str());
    }
  }
  return true;
}

rclcpp_action::GoalResponse MotionPrimitivesForwardController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteMotion::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received goal request");

  const auto & primitives = goal->trajectory.motions;

  if (primitives.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: no motion primitives provided.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (size_t i = 0; i < primitives.size(); ++i)
  {
    const auto & primitive = primitives[i];

    switch (primitive.type)
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

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPrimitivesForwardController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>>)
{
  RCLCPP_INFO(get_node()->get_logger(), "Canceling goal");

  std::lock_guard<std::mutex> guard(command_mutex_);
  reset_command_interfaces();
  robot_stopped_ = true;
  // send stop command immediately to the hw-interface
  (void)command_interfaces_[0].set_value(static_cast<double>(MotionType::STOP_MOTION));
  while (!moprim_queue_.empty())
  {  // clear the queue
    moprim_queue_.pop();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Waiting for Robot to stop...");
  while (execution_status_ != ExecutionState::STOPPED)
  {
    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  reset_command_interfaces();
  // send reset stop command to the hw-interface
  (void)command_interfaces_[0].set_value(static_cast<double>(MotionType::RESET_STOP));
  robot_stopped_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "Robot stopped, ready for new motion primitives.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPrimitivesForwardController::goal_accepted_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle)
{
  std::thread{std::bind(&MotionPrimitivesForwardController::execute_goal, this, goal_handle)}
    .detach();
}

void MotionPrimitivesForwardController::execute_goal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle)
{
  const auto & goal = goal_handle->get_goal();

  if (robot_stopped_)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Robot is stopped. Discarding the new command.");
    return;
  }

  const auto & primitives = goal->trajectory.motions;

  if (primitives.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Received goal with no motion primitives. Ignoring.");
    return;
  }

  size_t required_queue_space = primitives.size();
  // add sequence markers only if there are multiple primitives
  bool add_sequence_wrappers = primitives.size() > 1;
  if (add_sequence_wrappers)
  {
    required_queue_space += 2;  // start + end
  }

  // Check if queue has enough space
  if (moprim_queue_.size() + required_queue_space > queue_size_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received more motion primitives than queue can store. "
      "Current queue size: %zu, required space: %zu. "
      "Please increase the queue size in the controller parameters.",
      moprim_queue_.size(), required_queue_space);
    return;
  }

  // Add sequence start marker
  if (add_sequence_wrappers)
  {
    std::shared_ptr<MotionPrimitiveMsg> start_marker = std::make_shared<MotionPrimitiveMsg>();
    start_marker->type = MotionType::MOTION_SEQUENCE_START;
    moprim_queue_.push(start_marker);
  }

  // Add motion primitives to the queue
  for (const auto & primitive : primitives)
  {
    moprim_queue_.push(std::make_shared<MotionPrimitiveMsg>(primitive));
  }

  // Add sequence end marker
  if (add_sequence_wrappers)
  {
    std::shared_ptr<MotionPrimitiveMsg> end_marker = std::make_shared<MotionPrimitiveMsg>();
    end_marker->type = MotionType::MOTION_SEQUENCE_END;
    moprim_queue_.push(end_marker);
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "Accepted goal with %zu motion primitives%s.", primitives.size(),
    add_sequence_wrappers ? " (wrapped in sequence markers)" : "");

  ExecuteMotion::Result result;
  rclcpp::Rate rate(50);
  while (rclcpp::ok())
  {
    auto execution_status = static_cast<uint8_t>(std::round(state_interfaces_[0].get_value()));
    if (execution_status == ExecutionState::SUCCESS && was_executing_)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Execution completed successfully.");
      result.error_code = ExecuteMotion::Result::SUCCESSFUL;
      result.error_string = "Trajectory executed successfully";
      was_executing_ = false;
      break;
    }
    else if (execution_status == ExecutionState::ERROR)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Execution failed with an error.");
      result.error_code = ExecuteMotion::Result::FAILED;
      result.error_string = "Trajectory execution failed";
      break;
    }
    else if (execution_status == ExecutionState::STOPPED)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Execution stopped.");
      result.error_code = ExecuteMotion::Result::CANCELED;
      result.error_string = "Trajectory execution stopped";
      break;
    }
    rate.sleep();
  }

  goal_handle->succeed(std::make_shared<ExecuteMotion::Result>(result));
}
}  // namespace motion_primitives_forward_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_forward_controller::MotionPrimitivesForwardController,
  controller_interface::ControllerInterface)
