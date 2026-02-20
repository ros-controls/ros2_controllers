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

#include "motion_primitives_controllers/motion_primitives_from_trajectory_controller.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/helpers.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace motion_primitives_controllers
{
controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_init()
{
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Initializing Motion Primitives From Trajectory Controller");
  try
  {
    param_listener_ =
      std::make_shared<motion_primitives_from_trajectory_controller::ParamListener>(get_node());
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

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Configuring Motion Primitives From Trajectory Controller");

  params_ = param_listener_->get_params();
  tf_prefix_ = params_.tf_prefix;

  if (params_.local_joint_order.size() != 6)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error: Exactly 6 joints must be provided in local_joint_order!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.approximate_mode == "RDP_PTP")
  {
    approx_mode_ = ApproxMode::RDP_PTP;
  }
  else if (params_.approximate_mode == "RDP_LIN")
  {
    approx_mode_ = ApproxMode::RDP_LIN;
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error: Unknown approximate mode '%s'",
      params_.approximate_mode.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  use_time_not_vel_and_acc_ = params_.use_time_not_vel_and_acc;
  epsilon_joint_angle_ = params_.epsilon_joint_angle;
  epsilon_cart_position_ = params_.epsilon_cart_position;
  epsilon_cart_angle_ = params_.epsilon_cart_angle;
  blend_radius_percentage_ = params_.blend_radius_percentage;
  blend_radius_lower_limit_ = params_.blend_radius_lower_limit;
  blend_radius_upper_limit_ = params_.blend_radius_upper_limit;
  joint_vel_overwrite_ = params_.joint_vel_overwrite;
  joint_acc_overwrite_ = params_.joint_acc_overwrite;
  cart_vel_overwrite_ = params_.cart_vel_overwrite;
  cart_acc_overwrite_ = params_.cart_acc_overwrite;
  blend_radius_overwrite_ = params_.blend_radius_overwrite;

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

  planned_trajectory_publisher_ =
    get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", rclcpp::QoS(1));
  planned_poses_publisher_ =
    get_node()->create_publisher<geometry_msgs::msg::PoseArray>("~/planned_poses", rclcpp::QoS(1));
  motion_primitive_publisher_ =
    get_node()->create_publisher<control_msgs::msg::MotionPrimitiveSequence>(
      "~/approximated_motion_primitives", rclcpp::QoS(1));

  return MotionPrimitivesBaseController::on_configure(previous_state);
}

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  fk_client_ = std::make_shared<FKClient>();

  return MotionPrimitivesBaseController::on_activate(previous_state);
}

controller_interface::CallbackReturn MotionPrimitivesFromTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  action_server_.reset();
  fk_client_.reset();

  return MotionPrimitivesBaseController::on_deactivate(previous_state);
}

controller_interface::return_type MotionPrimitivesFromTrajectoryController::update(
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
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
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
            auto result = std::make_shared<FollowJTrajAction::Result>();
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

rclcpp_action::GoalResponse MotionPrimitivesFromTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
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

  if (robot_stop_requested_)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Robot requested to stop. Discarding the new command.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // TODO(mathias31415): Check if queue has enough space? Number of primitives not known here?

  if (goal->trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPrimitivesFromTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>>)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPrimitivesFromTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Processing accepted goal ...");

  auto trajectory_msg =
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

  sort_to_local_joint_order(trajectory_msg);

  // Publish planned trajectory
  planned_trajectory_publisher_->publish(*trajectory_msg);

  RCLCPP_INFO(
    get_node()->get_logger(), "Received trajectory with %zu points.",
    trajectory_msg->points.size());

  const auto & joint_names = trajectory_msg->joint_names;

  geometry_msgs::msg::PoseArray planned_poses_msg;
  planned_poses_msg.header.stamp = get_node()->now();
  planned_poses_msg.header.frame_id = "base";

  std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> planned_trajectory_data;
  planned_trajectory_data.reserve(trajectory_msg->points.size());
  std::vector<double> time_from_start;
  time_from_start.reserve(planned_trajectory_data.size());
  for (const auto & point : trajectory_msg->points)
  {
    approx_primitives_with_rdp::PlannedTrajectoryPoint pt;
    pt.time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    time_from_start.push_back(pt.time_from_start);
    pt.joint_positions = point.positions;
    try
    {
      pt.pose = fk_client_->computeFK(joint_names, point.positions, "base", "tool0");
      planned_poses_msg.poses.push_back(pt.pose);
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Tool0 pose: position (%.3f, %.3f, %.3f), orientation [%.3f, %.3f, %.3f, %.3f]",
        pt.pose.position.x, pt.pose.position.y, pt.pose.position.z, pt.pose.orientation.x,
        pt.pose.orientation.y, pt.pose.orientation.z, pt.pose.orientation.w);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "FK-Error: %s", e.what());
    }
    planned_trajectory_data.push_back(pt);
  }
  // Publish planned poses
  planned_poses_publisher_->publish(planned_poses_msg);

  control_msgs::msg::MotionPrimitiveSequence motion_sequence;
  switch (approx_mode_)
  {
    case ApproxMode::RDP_PTP:
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Approximating motion primitives with PTP motion primitives.");
      get_max_traj_joint_vel_and_acc(trajectory_msg, max_traj_joint_vel_, max_traj_joint_acc_);
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Max trajectory joint velocity: %.3f, Max trajectory joint acceleration: %.3f",
        max_traj_joint_vel_, max_traj_joint_acc_);
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Joint velocity override: %.3f, Joint acceleration override: %.3f", joint_vel_overwrite_,
        joint_acc_overwrite_);
      if (joint_vel_overwrite_ > 0.0)
      {
        max_traj_joint_vel_ = joint_vel_overwrite_;
      }
      if (joint_acc_overwrite_ > 0.0)
      {
        max_traj_joint_acc_ = joint_acc_overwrite_;
      }
      motion_sequence = approxPtpPrimitivesWithRDP(
        planned_trajectory_data, epsilon_joint_angle_, max_traj_joint_vel_, max_traj_joint_acc_,
        use_time_not_vel_and_acc_, blend_radius_overwrite_, blend_radius_percentage_,
        blend_radius_lower_limit_, blend_radius_upper_limit_);
      break;
    }
    case ApproxMode::RDP_LIN:
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Approximating motion primitives with LIN motion primitives.");
      get_max_traj_cart_vel_and_acc(
        planned_poses_msg, time_from_start, max_traj_cart_vel_, max_traj_cart_acc_);
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Max trajectory cartesian velocity: %.3f, Max trajectory cartesian acceleration: %.3f",
        max_traj_cart_vel_, max_traj_cart_acc_);
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Cartesian velocity override: %.3f, Cartesian acceleration override: %.3f",
        cart_vel_overwrite_, cart_acc_overwrite_);
      if (cart_vel_overwrite_ > 0.0)
      {
        max_traj_cart_vel_ = cart_vel_overwrite_;
      }
      if (cart_acc_overwrite_ > 0.0)
      {
        max_traj_cart_acc_ = cart_acc_overwrite_;
      }
      motion_sequence = approxLinPrimitivesWithRDP(
        planned_trajectory_data, epsilon_cart_position_, epsilon_cart_angle_, max_traj_cart_vel_,
        max_traj_cart_acc_, use_time_not_vel_and_acc_, blend_radius_overwrite_,
        blend_radius_percentage_, blend_radius_lower_limit_, blend_radius_upper_limit_);
      break;
    }
    default:
      RCLCPP_WARN(get_node()->get_logger(), "Unknown motion type.");
      break;
  }
  // Publish approximated motion primitives
  motion_primitive_publisher_->publish(motion_sequence);

  auto add_motions = [this](const control_msgs::msg::MotionPrimitiveSequence & moprim_sequence)
  {
    for (const auto & primitive : moprim_sequence.motions)
    {
      if (!moprim_queue_.push(primitive))
      {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to push motion primitive to queue.");
      }
    }
  };

  if (motion_sequence.motions.size() > 1)
  {
    MotionPrimitive start_marker;
    start_marker.type = static_cast<uint8_t>(MotionHelperType::MOTION_SEQUENCE_START);
    if (!moprim_queue_.push(start_marker))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Failed to push motion sequence start marker to queue.");
    }

    add_motions(motion_sequence);

    MotionPrimitive end_marker;
    end_marker.type = static_cast<uint8_t>(MotionHelperType::MOTION_SEQUENCE_END);
    if (!moprim_queue_.push(end_marker))
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to push motion sequence end marker to queue.");
    }
  }
  else
  {
    add_motions(motion_sequence);
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
    get_node()->get_logger(),
    "Reduced planned joint trajectory from %zu points to %zu motion primitives.",
    trajectory_msg->points.size(), motion_sequence.motions.size());
}

void MotionPrimitivesFromTrajectoryController::get_max_traj_joint_vel_and_acc(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory_msg, double & max_vel,
  double & max_acc)
{
  max_vel = 0.0;
  max_acc = 0.0;

  if (!trajectory_msg)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("MotionPrimitivesFromTrajectoryController"),
      "Received null trajectory pointer in get_max_traj_joint_vel_and_acc()");
    return;
  }

  for (const auto & point : trajectory_msg->points)
  {
    for (const auto & vel : point.velocities)
    {
      max_vel = std::max(max_vel, std::abs(vel));
    }

    for (const auto & acc : point.accelerations)
    {
      max_acc = std::max(max_acc, std::abs(acc));
    }
  }
}

void MotionPrimitivesFromTrajectoryController::get_max_traj_cart_vel_and_acc(
  const geometry_msgs::msg::PoseArray & planned_poses_msg,
  const std::vector<double> & time_from_start, double & max_vel, double & max_acc)
{
  max_vel = 0.0;
  max_acc = 0.0;

  const auto & poses = planned_poses_msg.poses;
  size_t n = poses.size();

  if (n < 2 || time_from_start.size() != n)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid input: expected at least 2 poses and time_from_start of same size, got %zu poses "
      "and %zu time values.",
      n, time_from_start.size());
    return;
  }

  std::vector<double> translational_velocities;

  for (size_t i = 1; i < n; ++i)
  {
    const auto & p1 = poses[i - 1].position;
    const auto & p2 = poses[i].position;

    double dt = time_from_start[i] - time_from_start[i - 1];
    if (dt <= 0.0)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid time difference between poses: %f", dt);
      continue;
    }

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double vel = dist / dt;
    translational_velocities.push_back(vel);

    max_vel = std::max(max_vel, vel);
  }

  for (size_t i = 1; i < translational_velocities.size(); ++i)
  {
    double dv = translational_velocities[i] - translational_velocities[i - 1];
    double dt = time_from_start[i + 1] - time_from_start[i];
    if (dt <= 0.0) continue;

    double acc = std::abs(dv / dt);
    max_acc = std::max(max_acc, acc);
  }
}

void MotionPrimitivesFromTrajectoryController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  std::vector<size_t> mapping_vector =
    mapping(trajectory_msg->joint_names, params_.local_joint_order);

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

  trajectory_msg->joint_names = params_.local_joint_order;
}

}  // namespace motion_primitives_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_controllers::MotionPrimitivesFromTrajectoryController,
  controller_interface::ControllerInterface)
