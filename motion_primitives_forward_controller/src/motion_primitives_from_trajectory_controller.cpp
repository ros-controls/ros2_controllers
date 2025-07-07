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

#include "motion_primitives_forward_controller/motion_primitives_from_trajectory_controller.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/helpers.hpp"

namespace motion_primitives_from_trajectory_controller
{
MotionPrimitivesFromTrajectoryController::MotionPrimitivesFromTrajectoryController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_init()
{
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Initializing Motion Primitives From Trajectory Controller");

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

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Configuring Motion Primitives From Trajectory Controller");

  params_ = param_listener_->get_params();

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

  fk_client_ = std::make_shared<FKClient>();

  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(
      &MotionPrimitivesFromTrajectoryController::goal_received_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &MotionPrimitivesFromTrajectoryController::goal_cancelled_callback, this,
      std::placeholders::_1),
    std::bind(
      &MotionPrimitivesFromTrajectoryController::goal_accepted_callback, this,
      std::placeholders::_1));

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesFromTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.command_interfaces.size());

  // Iterate over all command interfaces from the config yaml file
  for (const auto & interface_name : params_.command_interfaces)
  {
    command_interfaces_config.names.push_back("motion_primitive/" + interface_name);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotionPrimitivesFromTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params_.state_interfaces.size());

  // Iterate over all state interfaces from the config yaml file
  for (const auto & interface_name : params_.state_interfaces)
  {
    state_interfaces_config.names.push_back("motion_primitive/" + interface_name);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  moprim_queue_write_enabled_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces();
  action_server_.reset();
  fk_client_.reset();
  RCLCPP_DEBUG(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotionPrimitivesFromTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (cancel_requested_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Cancel requested, stopping execution.");
    cancel_requested_ = false;
    reset_command_interfaces();
    // send stop command immediately to the hw-interface
    (void)command_interfaces_[0].set_value(static_cast<double>(MotionType::STOP_MOTION));
    while (!moprim_queue_.empty())
    {  // clear the queue
      moprim_queue_.pop();
    }
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

      if (pending_action_goal_ && was_executing_)
      {
        was_executing_ = false;
        auto result = std::make_shared<FollowJTrajAction::Result>();
        result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
        result->error_string = "Motion primitives executed successfully";
        pending_action_goal_->succeed(result);
        pending_action_goal_.reset();
        RCLCPP_INFO(get_node()->get_logger(), "Motion primitives executed successfully.");
      }

      break;

    case ExecutionState::STOPPED:
      print_error_once_ = true;
      was_executing_ = false;

      if (pending_action_goal_)
      {
        auto result = std::make_shared<FollowJTrajAction::Result>();
        // result->error_code = FollowJTrajAction::Result::CANCELED;
        result->error_code = 69;
        result->error_string = "Motion primitives execution canceled";
        pending_action_goal_->succeed(result);
        pending_action_goal_.reset();
        RCLCPP_INFO(get_node()->get_logger(), "Motion primitives execution canceled.");
      }

      if (robot_stop_requested_)
      {
        // If the robot was stopped by a stop command, reset the command interfaces
        // to allow new motion primitives to be sent.
        reset_command_interfaces();
        (void)command_interfaces_[0].set_value(static_cast<double>(MotionType::RESET_STOP));
        robot_stop_requested_ = false;
        moprim_queue_write_enabled_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Robot stopped, ready for new motion primitives.");
      }
      break;

    case ExecutionState::ERROR:
      was_executing_ = false;

      if (pending_action_goal_)
      {
        auto result = std::make_shared<FollowJTrajAction::Result>();
        // result->error_code = FollowJTrajAction::Result::FAILED;
        result->error_code = 404;
        result->error_string = "Motion primitives execution failed";
        pending_action_goal_->succeed(result);
        pending_action_goal_.reset();
        RCLCPP_INFO(get_node()->get_logger(), "Motion primitives execution failed");
      }

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

  if (!moprim_queue_write_enabled_ && !cancel_requested_)
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
          // all primitives read, queue ready to get filled with new primitives
          moprim_queue_write_enabled_ = true;
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
void MotionPrimitivesFromTrajectoryController::reset_command_interfaces()
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
bool MotionPrimitivesFromTrajectoryController::set_command_interfaces()
{
  // Get the oldest message from the queue
  std::shared_ptr<MotionPrimitive> current_moprim = moprim_queue_.front();
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
    if (
      current_moprim->type == static_cast<uint8_t>(MotionType::CIRCULAR_CARTESIAN) &&
      current_moprim->poses.size() == 2)
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

rclcpp_action::GoalResponse MotionPrimitivesFromTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  if (robot_stop_requested_)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Robot requested to stop. Discarding the new command.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!moprim_queue_write_enabled_)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Queue is not ready to write. Discarding the new command.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // TODO(mathias31415): Check if first trajectory point matches the current robot state

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPrimitivesFromTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>>)
{
  cancel_requested_ = true;
  moprim_queue_write_enabled_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPrimitivesFromTrajectoryController::goal_accepted_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Processing accepted goal ...");

  pending_action_goal_ = goal_handle;  // Store the goal handle for later result feedback

  auto trajectory_msg =
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

  sort_to_local_joint_order(trajectory_msg);

  RCLCPP_INFO(
    get_node()->get_logger(), "Received trajectory with %zu points.",
    trajectory_msg->points.size());

  const auto & joint_names = trajectory_msg->joint_names;

  std::ostringstream oss;
  for (const auto & name : joint_names)
  {
    oss << name << " ";
  }
  RCLCPP_INFO(get_node()->get_logger(), "Received joint names: %s", oss.str().c_str());

  // Get endefector pose for each trajectory point
  for (const auto & point : trajectory_msg->points)
  {
    try
    {
      auto tool0_pose = fk_client_->computeFK(joint_names, point.positions, "base_link", "tool0");
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Tool0 pose: position (%.3f, %.3f, %.3f), orientation [%.3f, %.3f, %.3f, %.3f]",
        tool0_pose.position.x, tool0_pose.position.y, tool0_pose.position.z,
        tool0_pose.orientation.x, tool0_pose.orientation.y, tool0_pose.orientation.z,
        tool0_pose.orientation.w);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "FK-Error: %s", e.what());
    }
  }

  std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> planned_trajectory_data;
  planned_trajectory_data.reserve(trajectory_msg->points.size());
  for (const auto & point : trajectory_msg->points)
  {
    approx_primitives_with_rdp::PlannedTrajectoryPoint pt;
    pt.time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    pt.joint_positions = point.positions;
    // pt.pose = fk_client_->computeFK(trajectory_msg->joint_names, point.positions, "base",
    // "tool0").value_or(Pose());
    planned_trajectory_data.push_back(pt);
  }
  industrial_robot_motion_interfaces::msg::MotionSequence motion_sequence;
  double rdp_epsilon = 0.001;
  bool use_time_not_vel_and_acc = true;
  switch (approx_type_)
  {
    case MotionType::LINEAR_JOINT:
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Approximating motion primitives with PTP motion primitives.");
      motion_sequence =
        approxPtpPrimitivesWithRDP(planned_trajectory_data, rdp_epsilon, use_time_not_vel_and_acc);

      break;
    }
    case MotionType::LINEAR_CARTESIAN:
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Approximating motion primitives with LIN motion primitives.");

      RCLCPP_WARN(get_node()->get_logger(), "Not implemented yet.");
      break;
    }
    default:
      RCLCPP_WARN(get_node()->get_logger(), "Unknown motion type.");
      break;
  }

  // // dummy primitives
  // MotionPrimitive moprim1;
  // moprim1.type = MotionPrimitive::LINEAR_JOINT;
  // moprim1.blend_radius = 0.0;
  // moprim1.joint_positions = {1.57, -1.57, 1.57, -1.57, -1.57, -1.57};
  // industrial_robot_motion_interfaces::msg::MotionArgument arg_time1;
  // arg_time1.argument_name = "move_time";
  // arg_time1.argument_value = 2.0;
  // moprim1.additional_arguments.push_back(arg_time1);

  // MotionPrimitive moprim2;
  // moprim2.type = MotionPrimitive::LINEAR_JOINT;
  // moprim2.blend_radius = 0.0;
  // moprim2.joint_positions = {0.9, -1.57, 1.57, -1.57, -1.57, -1.57};
  // industrial_robot_motion_interfaces::msg::MotionArgument arg_time2;
  // arg_time2.argument_name = "move_time";
  // arg_time2.argument_value = 2.0;
  // moprim2.additional_arguments.push_back(arg_time2);

  // industrial_robot_motion_interfaces::msg::MotionSequence motion_sequence;
  // motion_sequence.motions.push_back(moprim1);
  // motion_sequence.motions.push_back(moprim2);

  auto add_motions =
    [this](const industrial_robot_motion_interfaces::msg::MotionSequence & moprim_sequence)
  {
    for (const auto & primitive : moprim_sequence.motions)
    {
      moprim_queue_.push(std::make_shared<MotionPrimitive>(primitive));
    }
  };

  if (motion_sequence.motions.size() > 1)
  {
    std::shared_ptr<MotionPrimitive> start_marker = std::make_shared<MotionPrimitive>();
    start_marker->type = static_cast<uint8_t>(MotionType::MOTION_SEQUENCE_START);
    moprim_queue_.push(start_marker);

    add_motions(motion_sequence);

    std::shared_ptr<MotionPrimitive> end_marker = std::make_shared<MotionPrimitive>();
    end_marker->type = static_cast<uint8_t>(MotionType::MOTION_SEQUENCE_END);
    moprim_queue_.push(end_marker);
  }
  else
  {
    add_motions(motion_sequence);
  }
  moprim_queue_write_enabled_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Reduced planned joint trajectory from %zu points to %zu motion primitives.",
    trajectory_msg->points.size(), motion_sequence.motions.size());
}

// TODO(mathias31415): read local_joint_order from param instead
void MotionPrimitivesFromTrajectoryController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  const std::vector<std::string> local_joint_order = {"shoulder_pan_joint", "shoulder_lift_joint",
                                                      "elbow_joint",        "wrist_1_joint",
                                                      "wrist_2_joint",      "wrist_3_joint"};

  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, local_joint_order);

  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & map_indices) -> std::vector<double>
  {
    if (to_remap.empty()) return to_remap;

    if (to_remap.size() != map_indices.size())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
      return to_remap;
    }

    std::vector<double> output(map_indices.size(), 0.0);

    for (size_t index = 0; index < map_indices.size(); ++index)
    {
      size_t map_index = map_indices[index];
      if (map_index < to_remap.size())
      {
        output[index] = to_remap[map_index];
      }
    }

    return output;
  };

  for (auto & point : trajectory_msg->points)
  {
    point.positions = remap(point.positions, mapping_vector);

    if (!point.velocities.empty()) point.velocities = remap(point.velocities, mapping_vector);

    if (!point.accelerations.empty())
      point.accelerations = remap(point.accelerations, mapping_vector);

    if (!point.effort.empty()) point.effort = remap(point.effort, mapping_vector);
  }
}

}  // namespace motion_primitives_from_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_from_trajectory_controller::MotionPrimitivesFromTrajectoryController,
  controller_interface::ControllerInterface)
