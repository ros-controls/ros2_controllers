// Copyright 2024 ros2_control Development Team
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

#include "multi_time_trajectory_controller/multi_time_trajectory_controller.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <controller_interface/controller_interface_base.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "control_msgs/msg/axis_trajectory.hpp"
#include "control_msgs/msg/axis_trajectory_point.hpp"
#include "control_msgs/msg/multi_axis_trajectory.hpp"
#include "eigen3/Eigen/Eigen"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/create_server.hpp"

#include "controller_interface/helpers.hpp"

namespace multi_time_trajectory_controller
{

control_msgs::msg::AxisTrajectoryPoint emptyTrajectoryPoint()
{
  control_msgs::msg::AxisTrajectoryPoint atp;
  atp.position = std::numeric_limits<double>::quiet_NaN();
  atp.velocity = std::numeric_limits<double>::quiet_NaN();
  atp.acceleration = std::numeric_limits<double>::quiet_NaN();
  atp.effort = std::numeric_limits<double>::quiet_NaN();
  atp.time_from_start = rclcpp::Duration(0, 0);
  return atp;
}

MultiTimeTrajectoryController::MultiTimeTrajectoryController()
: controller_interface::ControllerInterface(), dof_(0)
{
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    joint_limiter_loader_ = std::make_shared<pluginlib::ClassLoader<JointLimiter>>(
      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>");
    RCLCPP_DEBUG(get_node()->get_logger(), "Available joint limiter classes:");
    for (const auto & available_class : joint_limiter_loader_->getDeclaredClasses())
    {
      RCLCPP_DEBUG(get_node()->get_logger(), "  %s", available_class.c_str());
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();
  if (!urdf.empty())
  {
    urdf::Model model;
    if (!model.initString(urdf))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description!");
      return CallbackReturn::ERROR;
    }
    else
    {
      /// initialize the URDF model and update the axis angles wraparound vector
      // Configure axis position error normalization (angle_wraparound)
      axis_angle_wraparound_.resize(params_.axes.size(), false);
      for (size_t i = 0; i < params_.axes.size(); ++i)
      {
        auto urdf_joint = model.getJoint(params_.axes[i]);
        if (urdf_joint && urdf_joint->type == urdf::Joint::CONTINUOUS)
        {
          RCLCPP_DEBUG(
            get_node()->get_logger(), "joint '%s' is of type continuous, use angle_wraparound.",
            params_.axes[i].c_str());
          axis_angle_wraparound_[i] = true;
        }
        // do nothing if joint is not found in the URDF
      }
      RCLCPP_DEBUG(get_node()->get_logger(), "Successfully parsed URDF file");
    }
  }
  else
  {
    // empty URDF is used for some tests
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  // Initialize joint limits
  joint_limits_.resize(params_.axes.size());

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MultiTimeTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (dof_ == 0)
  {
    fprintf(
      stderr,
      "During ros2_control interface configuration, degrees of freedom is not valid;"
      " it should be positive. Actual DOF is %zu\n",
      dof_);
    std::exit(EXIT_FAILURE);
  }
  conf.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & axis_name : command_axis_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(axis_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MultiTimeTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  if (params_.state_interfaces.size() == 0)
  {
    conf.type = controller_interface::interface_configuration_type::NONE;
    return conf;
  }
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & axis_name : params_.axes)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(axis_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::return_type MultiTimeTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(params_);
    // update the PID gains
    // variable use_closed_loop_pid_adapter_ is updated in on_configure only
    if (use_closed_loop_pid_adapter_)
    {
      update_pids();
    }
  }

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (
    current_external_msg != *new_external_msg &&
    (*(rt_has_pending_goal_.readFromRT()) && !active_goal) == false)
  {
    sort_to_local_axis_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg, joint_limits_, period, time);
  }

  // current state update - bail if can't read hardware state
  if (!read_state_from_hardware(state_current_)) return controller_interface::return_type::OK;

  bool first_sample = false;
  std::vector<int> segment_start(dof_, -1);
  // currently carrying out a trajectory
  if (has_active_trajectory())
  {
    // if sampling the first time, set the point before you sample
    if (!traj_external_point_ptr_->is_sampled_already())
    {
      first_sample = true;

      // Reset Ruckig vel/accel/jerk smoothing
      //       (*traj_point_active_ptr_)->reset_ruckig_smoothing();

      if (params_.open_loop_control)
      {
        auto reset_flags = reset_dofs_flags_.readFromRT();
        for (size_t i = 0; i < dof_; ++i)
        {
          if (reset_flags->at(i).reset)
          {
            last_commanded_state_[i].position = std::isnan(reset_flags->at(i).position)
                                                  ? state_current_[i].position
                                                  : reset_flags->at(i).position;
            RCLCPP_INFO_STREAM(
              get_node()->get_logger(), command_axis_names_[i]
                                          << ": last commanded state position reset to "
                                          << last_commanded_state_[i].position);
            if (has_velocity_state_interface_)
            {
              last_commanded_state_[i].velocity = std::isnan(reset_flags->at(i).velocity)
                                                    ? state_current_[i].velocity
                                                    : reset_flags->at(i).velocity;
            }
            if (has_acceleration_state_interface_)
            {
              last_commanded_state_[i].acceleration = std::isnan(reset_flags->at(i).acceleration)
                                                        ? state_current_[i].acceleration
                                                        : reset_flags->at(i).acceleration;
            }

            reset_flags->at(i).reset = false;  // reset flag in the buffer for one-shot execution
          }
        }

        if (last_commanded_time_.seconds() == 0.0)
        {
          last_commanded_time_ = time;
        }
        traj_external_point_ptr_->set_point_before_trajectory_msg(
          time, last_commanded_state_, axis_angle_wraparound_);
      }
      else
      {
        traj_external_point_ptr_->set_point_before_trajectory_msg(
          time, state_current_, axis_angle_wraparound_);
      }
    }

    // find segment for current timestamp
    std::vector<TrajectoryPointConstIter> start_segment_itrs, end_segment_itrs;
    start_segment_itrs.resize(dof_);
    end_segment_itrs.resize(dof_);
    auto const valid_points = traj_external_point_ptr_->sample(
      time, interpolation_method_, state_desired_, start_segment_itrs, end_segment_itrs, period,
      joint_limiter_, splines_state_, ruckig_state_, ruckig_input_state_,
      params_.hold_last_velocity);

    compute_error(state_error_, state_current_, state_desired_);

    for (std::size_t axis_index = 0; axis_index < dof_; ++axis_index)
    {
      if (valid_points[axis_index])
      {
        auto const & start_segment_itr = start_segment_itrs[axis_index];
        auto const & end_segment_itr = end_segment_itrs[axis_index];
        const rclcpp::Time traj_start = traj_external_point_ptr_->start_time();

        if (
          (start_segment_itr == end_segment_itr) &&
          (std::distance(traj_external_point_ptr_->begin(axis_index), start_segment_itr) == 0))
        {
          segment_start[axis_index] = -1;
        }
        else
        {
          segment_start[axis_index] =
            (int)(std::distance(traj_external_point_ptr_->begin(axis_index), start_segment_itr));
        }

        // this is the time instance
        // - started with the first segment: when the first point will be reached (in the future)
        // - later: when the point of the current segment was reached
        const rclcpp::Time segment_time_from_start =
          traj_start + start_segment_itr->time_from_start;
        // time_difference is
        // - negative until first point is reached
        // - counting from zero to time_from_start of next point
        double time_difference = time.seconds() - segment_time_from_start.seconds();
        bool tolerance_violated_while_moving = false;
        bool outside_goal_tolerance = false;
        bool within_goal_time = true;

        // whether we are before the last point for the trajectory associated with axis_index
        bool const before_last_point = end_segment_itr != traj_external_point_ptr_->end(axis_index);

        // have we reached the end, are not holding position, and is a timeout configured?
        // Check independently of other tolerances
        if (
          !before_last_point && *(rt_is_holding_.readFromRT()) == false && cmd_timeout_ > 0.0 &&
          time_difference > cmd_timeout_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to command timeout");

          traj_msg_external_point_ptr_.reset();
          traj_msg_external_point_ptr_.initRT(set_hold_position());
        }

        // Check state/goal tolerance
        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if (
          (before_last_point || first_sample) && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, axis_index, default_tolerances_.state_tolerance[axis_index],
            true /* show_errors */))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, axis_index, default_tolerances_.goal_state_tolerance[axis_index],
            false /* show_errors */))
        {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            if (time_difference > default_tolerances_.goal_time_tolerance)
            {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(
                state_error_, axis_index, default_tolerances_.goal_state_tolerance[axis_index],
                true /* show_errors */);
            }
          }
        }

        // set values for next hardware write() if tolerance is met
        if (!tolerance_violated_while_moving && within_goal_time)
        {
          double tmp_command = 0;
          if (use_closed_loop_pid_adapter_)
          {
            // Update PID
            tmp_command = (state_desired_[axis_index].velocity * ff_velocity_scale_[axis_index]) +
                          pids_[axis_index]->computeCommand(
                            state_error_[axis_index].position, state_error_[axis_index].velocity,
                            (uint64_t)period.nanoseconds());
          }

          // set values for next hardware write()
          if (has_position_command_interface_)
          {
            axis_command_interface_[0][axis_index].get().set_value(
              state_desired_[axis_index].position);
          }
          if (has_velocity_command_interface_)
          {
            if (use_closed_loop_pid_adapter_)
            {
              axis_command_interface_[1][axis_index].get().set_value(tmp_command);
            }
            else
            {
              axis_command_interface_[1][axis_index].get().set_value(
                state_desired_[axis_index].velocity);
            }
          }
          if (has_acceleration_command_interface_)
          {
            axis_command_interface_[2][axis_index].get().set_value(
              state_desired_[axis_index].acceleration);
          }
          if (has_effort_command_interface_)
          {
            axis_command_interface_[3][axis_index].get().set_value(tmp_command);
          }

          // store the previous command. Used in open-loop control mode
          last_commanded_state_ = state_desired_;
          last_commanded_time_ = time;
        }

        if (active_goal)
        {
          // send feedback
          auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
          feedback->header.stamp = time;
          feedback->axis_names = params_.axes;

          feedback->actual_points = state_current_;
          feedback->desired_points = state_desired_;
          feedback->errors = state_error_;
          active_goal->setFeedback(feedback);

          // check abort
          if (tolerance_violated_while_moving)
          {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
            result->set__error_string("Aborted due to path tolerance violation");
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");

            traj_msg_external_point_ptr_.reset();
            traj_msg_external_point_ptr_.initRT(set_hold_position());
          }
          // check goal tolerance
          else if (!before_last_point)
          {
            if (!outside_goal_tolerance)
            {
              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
              result->set__error_string("Goal successfully reached!");
              active_goal->setSucceeded(result);
              // TODO(matthew-reynolds): Need a lock-free write here
              // See https://github.com/ros-controls/ros2_controllers/issues/168
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              rt_has_pending_goal_.writeFromNonRT(false);

              RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");

              traj_msg_external_point_ptr_.reset();
              traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
            }
            else if (!within_goal_time)
            {
              const std::string error_string = "Aborted due to goal_time_tolerance exceeding by " +
                                               std::to_string(time_difference) + " seconds";

              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
              result->set__error_string(error_string);
              active_goal->setAborted(result);
              // TODO(matthew-reynolds): Need a lock-free write here
              // See https://github.com/ros-controls/ros2_controllers/issues/168
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              rt_has_pending_goal_.writeFromNonRT(false);

              RCLCPP_WARN(get_node()->get_logger(), "%s", error_string.c_str());

              traj_msg_external_point_ptr_.reset();
              traj_msg_external_point_ptr_.initRT(set_hold_position());
            }
          }
        }
        else if (tolerance_violated_while_moving && *(rt_has_pending_goal_.readFromRT()) == false)
        {
          // we need to ensure that there is no pending goal -> we get a race condition otherwise
          RCLCPP_ERROR(
            get_node()->get_logger(), "Holding position due to state tolerance violation");

          traj_msg_external_point_ptr_.reset();
          traj_msg_external_point_ptr_.initRT(set_hold_position());
        }
        else if (
          !before_last_point && !within_goal_time && *(rt_has_pending_goal_.readFromRT()) == false)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Exceeded goal_time_tolerance: holding position...");

          traj_msg_external_point_ptr_.reset();
          traj_msg_external_point_ptr_.initRT(set_hold_position());
        }
        // else, run another cycle while waiting for outside_goal_tolerance
        // to be satisfied (will stay in this state until new message arrives)
        // or outside_goal_tolerance violated within the goal_time_tolerance
      }
    }
  }

  publish_state(time, first_sample, segment_start);
  return controller_interface::return_type::OK;
}

bool MultiTimeTrajectoryController::read_state_from_hardware(std::vector<TrajectoryPoint> & states)
{
  // start by emptying all states
  std::fill(states.begin(), states.end(), emptyTrajectoryPoint());

  if (params_.use_feedback)
  {
    if (dof_ != 6)
    {
      throw std::runtime_error("must have 6 degrees of freedom to use feedback");
    }
    std::array<double, 3> orientation_angles;
    const auto measured_state = *(feedback_.readFromRT());
    if (!measured_state) return false;
    last_odom_feedback_ = *measured_state;

    tf2::Quaternion measured_q;

    if (
      std::isnan(measured_state->pose.pose.orientation.w) ||
      std::isnan(measured_state->pose.pose.orientation.x) ||
      std::isnan(measured_state->pose.pose.orientation.y) ||
      std::isnan(measured_state->pose.pose.orientation.z))
    {
      // if any of the orientation is NaN, revert to previous orientation
      measured_q.setRPY(states[3].position, states[4].position, states[5].position);
    }
    else
    {
      tf2::fromMsg(measured_state->pose.pose.orientation, measured_q);
    }
    tf2::Matrix3x3 m(measured_q);
    m.getRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);

    // Assign values from the hardware
    // Position states always exist
    // if any measured position is NaN, keep previous value
    states[0].position = std::isnan(measured_state->pose.pose.position.x)
                           ? states[0].position
                           : measured_state->pose.pose.position.x;
    states[1].position = std::isnan(measured_state->pose.pose.position.y)
                           ? states[1].position
                           : measured_state->pose.pose.position.y;
    states[2].position = std::isnan(measured_state->pose.pose.position.z)
                           ? states[2].position
                           : measured_state->pose.pose.position.z;
    states[3].position = orientation_angles[0];
    states[4].position = orientation_angles[1];
    states[5].position = orientation_angles[2];

    // Convert measured twist which is in body frame to world frame since CTG/JTC expects state
    // in world frame

    Eigen::Quaterniond q_body_in_world(
      measured_q.w(), measured_q.x(), measured_q.y(), measured_q.z());

    // if any measured linear velocity is NaN, set to zero
    Eigen::Vector3d linear_vel_body(
      std::isnan(measured_state->twist.twist.linear.x) ? 0.0 : measured_state->twist.twist.linear.x,
      std::isnan(measured_state->twist.twist.linear.y) ? 0.0 : measured_state->twist.twist.linear.y,
      std::isnan(measured_state->twist.twist.linear.z) ? 0.0
                                                       : measured_state->twist.twist.linear.z);
    auto linear_vel_world = q_body_in_world * linear_vel_body;

    // if any measured angular velocity is NaN, set to zero
    Eigen::Vector3d angular_vel_body(
      std::isnan(measured_state->twist.twist.angular.x) ? 0.0
                                                        : measured_state->twist.twist.angular.x,
      std::isnan(measured_state->twist.twist.angular.y) ? 0.0
                                                        : measured_state->twist.twist.angular.y,
      std::isnan(measured_state->twist.twist.angular.z) ? 0.0
                                                        : measured_state->twist.twist.angular.z);
    auto angular_vel_world = q_body_in_world * angular_vel_body;

    states[0].velocity = linear_vel_world[0];
    states[1].velocity = linear_vel_world[1];
    states[2].velocity = linear_vel_world[2];
    states[3].velocity = angular_vel_world[0];
    states[4].velocity = angular_vel_world[1];
    states[5].velocity = angular_vel_world[2];

    for (std::size_t i = 0; i < 6; ++i)
    {
      states[i].acceleration = std::numeric_limits<double>::quiet_NaN();
    }
    return true;
  }
  else
  {
    // Assign values from the hardware
    if (has_position_state_interface_)
    {
      assign_positions_from_interface(states, axis_state_interface_[0]);
      // velocity and acceleration states are optional
      if (has_velocity_state_interface_)
      {
        assign_velocities_from_interface(states, axis_state_interface_[1]);
        // Acceleration is used only in combination with velocity
        if (has_acceleration_state_interface_)
        {
          assign_accelerations_from_interface(states, axis_state_interface_[2]);
        }
      }
    }
  }
  return true;
}

bool MultiTimeTrajectoryController::read_state_from_command_interfaces(
  std::vector<TrajectoryPoint> & states)
{
  // start by emptying all states
  std::fill(states.begin(), states.end(), emptyTrajectoryPoint());
  bool has_values = true;

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(), [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as state
  if (has_position_command_interface_ && interface_has_values(axis_command_interface_[0]))
  {
    assign_positions_from_interface(states, axis_command_interface_[0]);
  }
  else
  {
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    if (has_velocity_command_interface_ && interface_has_values(axis_command_interface_[1]))
    {
      assign_velocities_from_interface(states, axis_command_interface_[1]);
    }
    else
    {
      has_values = false;
    }
  }
  // Acceleration is used only in combination with velocity
  if (has_acceleration_state_interface_)
  {
    if (has_acceleration_command_interface_ && interface_has_values(axis_command_interface_[2]))
    {
      assign_accelerations_from_interface(states, axis_command_interface_[2]);
    }
    else
    {
      has_values = false;
    }
  }

  return has_values;
}

bool MultiTimeTrajectoryController::read_commands_from_command_interfaces(
  std::vector<TrajectoryPoint> & commands)
{
  // start by emptying all commands
  std::fill(commands.begin(), commands.end(), emptyTrajectoryPoint());

  bool has_values = true;

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(), [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as command.
  if (has_position_command_interface_)
  {
    if (interface_has_values(axis_command_interface_[0]))
    {
      assign_positions_from_interface(commands, axis_command_interface_[0]);
    }
    else
    {
      has_values = false;
    }
  }
  if (has_velocity_command_interface_)
  {
    if (interface_has_values(axis_command_interface_[1]))
    {
      assign_velocities_from_interface(commands, axis_command_interface_[1]);
    }
    else
    {
      has_values = false;
    }
  }
  if (has_acceleration_command_interface_)
  {
    if (interface_has_values(axis_command_interface_[2]))
    {
      assign_accelerations_from_interface(commands, axis_command_interface_[2]);
    }
    else
    {
      has_values = false;
    }
  }

  return has_values;
}

void MultiTimeTrajectoryController::query_state_service(
  const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
  std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response)
{
  const auto logger = get_node()->get_logger();
  // Preconditions
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(logger, "Can't sample trajectory. Controller is not active.");
    response->success = false;
    return;
  }
  const auto active_goal = *rt_active_goal_.readFromRT();
  response->name = params_.axes;
  std::vector<control_msgs::msg::AxisTrajectoryPoint> state_requested = state_current_;
  if (has_active_trajectory())
  {
    std::vector<TrajectoryPointConstIter> start_segment_itrs, end_segment_itrs;
    start_segment_itrs.resize(dof_);
    end_segment_itrs.resize(dof_);
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
    auto const valid_points = traj_external_point_ptr_->sample(
      static_cast<rclcpp::Time>(request->time), interpolation_method_, state_requested,
      start_segment_itrs, end_segment_itrs, period, joint_limiter_, splines_state_, ruckig_state_,
      ruckig_input_state_, params_.hold_last_velocity);
    response->success =
      std::all_of(valid_points.begin(), valid_points.end(), [](bool b) { return b; });
    // If the requested sample time precedes the trajectory finish time respond as failure
    if (response->success)
    {
      bool any_is_end = false;
      for (std::size_t i = 0; i < dof_; ++i)
      {
        if (end_segment_itrs[i] == traj_external_point_ptr_->end(i))
        {
          any_is_end = true;
        }
      }
      if (any_is_end)
      {
        RCLCPP_ERROR(logger, "Requested sample time precedes the current trajectory end time.");
        response->success = false;
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(
        logger, "Requested sample time is earlier than the current trajectory start time.");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Currently there is no valid trajectory instance.");
    response->success = false;
    return;
  }
  response->position.clear();
  response->velocity.clear();
  response->acceleration.clear();
  for (std::size_t i = 0; i < dof_; ++i)
  {
    response->position.push_back(state_requested[i].position);
    response->velocity.push_back(state_requested[i].velocity);
    response->acceleration.push_back(state_requested[i].acceleration);
  }
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not get param listener during configure");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // get degrees of freedom
  dof_ = params_.axes.size();

  // size all state vectors appropriately
  state_current_.resize(dof_, emptyTrajectoryPoint());
  command_current_.resize(dof_, emptyTrajectoryPoint());
  state_desired_.resize(dof_, emptyTrajectoryPoint());
  state_error_.resize(dof_, emptyTrajectoryPoint());
  splines_state_.resize(dof_, emptyTrajectoryPoint());
  ruckig_state_.resize(dof_, emptyTrajectoryPoint());
  ruckig_input_state_.resize(dof_, emptyTrajectoryPoint());
  axis_angle_wraparound_.resize(dof_, false);

  // TODO(destogl): why is this here? Add comment or move
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  if (params_.axes.empty())
  {
    RCLCPP_ERROR(logger, "'axes' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  command_axis_names_ = params_.command_axes;

  if (command_axis_names_.empty())
  {
    command_axis_names_ = params_.axes;
    RCLCPP_INFO(
      logger, "No specific axis names are used for command interfaces. Using 'axes' parameter.");
  }
  else if (command_axis_names_.size() != params_.axes.size())
  {
    RCLCPP_ERROR(
      logger, "'command_axis_names' parameter has to have the same size as 'axes' parameter.");
    return CallbackReturn::FAILURE;
  }

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  axis_command_interface_.resize(allowed_interface_types_.size());

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // then use also PID adapter
  use_closed_loop_pid_adapter_ =
    (has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
     !params_.open_loop_control) ||
    has_effort_command_interface_;

  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(dof_);
    ff_velocity_scale_.resize(dof_);

    update_pids();
  }

  // Initialize joint limits
  if (!params_.joint_limiter_type.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Using joint limiter plugin: '%s'",
      params_.joint_limiter_type.c_str());
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_->createUnmanagedInstance(params_.joint_limiter_type));
    joint_limiter_->init(command_axis_names_, get_node());
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Not using joint limiter plugin as none defined.");
  }

  if (params_.state_interfaces.empty() && !params_.use_feedback)
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty and not using feedback.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used. Still, for this is OK.
  axis_state_interface_.resize(allowed_interface_types_.size());

  has_position_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  // Validation of combinations of state and velocity together have to be done
  // here because the parameter validators only deal with each parameter
  // separately.
  if (
    has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'velocity' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // effort is always used alone so no need for size check
  if (
    has_effort_command_interface_ &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'effort' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "Command interfaces are [%s] and state interfaces are [%s].",
    get_interface_list(params_.command_interfaces).c_str(),
    get_interface_list(params_.state_interfaces).c_str());

  // parse remaining parameters
  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ =
    joint_trajectory_controller::interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    joint_trajectory_controller::interpolation_methods::InterpolationMethodMap
      .at(interpolation_method_)
      .c_str());

  // prepare hold_position_msg
  init_hold_position_msg();

  // create subscriber and publishers
  axis_command_subscriber_ =
    get_node()->create_subscription<control_msgs::msg::MultiAxisTrajectory>(
      "~/axis_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&MultiTimeTrajectoryController::topic_callback, this, std::placeholders::_1));

  publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.axis_names = params_.axes;
  state_publisher_->msg_.references.resize(dof_);
  state_publisher_->msg_.feedbacks.resize(dof_);
  state_publisher_->msg_.errors.resize(dof_);
  state_publisher_->msg_.outputs.resize(dof_);

  state_publisher_->unlock();

  splines_output_pub_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/splines_output", rclcpp::SystemDefaultsQoS());
  splines_output_publisher_ = std::make_unique<StatePublisher>(splines_output_pub_);

  splines_output_publisher_->lock();
  splines_output_publisher_->msg_.axis_names = command_axis_names_;
  splines_output_publisher_->msg_.references.resize(dof_);
  splines_output_publisher_->msg_.feedbacks.resize(dof_);
  splines_output_publisher_->msg_.errors.resize(dof_);
  splines_output_publisher_->msg_.outputs.resize(dof_);
  splines_output_publisher_->unlock();

  ruckig_input_pub_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/ruckig_input_current", rclcpp::SystemDefaultsQoS());
  ruckig_input_publisher_ = std::make_unique<StatePublisher>(ruckig_input_pub_);

  ruckig_input_publisher_->lock();
  ruckig_input_publisher_->msg_.axis_names = command_axis_names_;
  ruckig_input_publisher_->msg_.references.resize(dof_);
  ruckig_input_publisher_->msg_.feedbacks.resize(dof_);
  ruckig_input_publisher_->msg_.errors.resize(dof_);
  ruckig_input_publisher_->msg_.outputs.resize(dof_);
  ruckig_input_publisher_->unlock();

  ruckig_input_target_pub_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/ruckig_input_target", rclcpp::SystemDefaultsQoS());
  ruckig_input_target_publisher_ = std::make_unique<StatePublisher>(ruckig_input_target_pub_);

  ruckig_input_target_publisher_->lock();
  ruckig_input_target_publisher_->msg_.axis_names = command_axis_names_;
  ruckig_input_target_publisher_->msg_.references.resize(dof_);
  ruckig_input_target_publisher_->msg_.feedbacks.resize(dof_);
  ruckig_input_target_publisher_->msg_.errors.resize(dof_);
  ruckig_input_target_publisher_->msg_.outputs.resize(dof_);
  ruckig_input_target_publisher_->unlock();

  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&MultiTimeTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&MultiTimeTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&MultiTimeTrajectoryController::goal_accepted_callback, this, _1));

  query_state_srv_ = get_node()->create_service<control_msgs::srv::QueryTrajectoryState>(
    std::string(get_node()->get_name()) + "/query_state",
    std::bind(&MultiTimeTrajectoryController::query_state_service, this, _1, _2));

  std::vector<ResetDofsData> reset_flags;
  reset_flags.resize(
    dof_, {false, std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});
  reset_dofs_flags_.writeFromNonRT(reset_flags);

  // topics QoS
  auto qos_best_effort_history_depth_one = rclcpp::SystemDefaultsQoS();
  qos_best_effort_history_depth_one.keep_last(1);
  qos_best_effort_history_depth_one.best_effort();
  auto subscribers_reliable_qos = rclcpp::SystemDefaultsQoS();
  subscribers_reliable_qos.keep_all();
  subscribers_reliable_qos.reliable();

  // Reference Subscribers (reliable channel also for updates not to be missed)
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", qos_best_effort_history_depth_one,
    std::bind(&MultiTimeTrajectoryController::reference_callback, this, std::placeholders::_1));
  ref_subscriber_reliable_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference_reliable", subscribers_reliable_qos,
    std::bind(&MultiTimeTrajectoryController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->axis_names = params_.axes;
  msg->axis_trajectories.resize(dof_);
  for (std::size_t i = 0; i < dof_; ++i)
  {
    msg->axis_trajectories[i].axis_points.resize(1, emptyTrajectoryPoint());
  }

  // Odometry feedback
  auto feedback_callback = [&](const std::shared_ptr<ControllerFeedbackMsg> feedback_msg) -> void
  { feedback_.writeFromNonRT(feedback_msg); };
  feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
    "~/feedback", qos_best_effort_history_depth_one, feedback_callback);
  // initialize feedback to null pointer since it is used to determine if we have valid data or not
  feedback_.writeFromNonRT(nullptr);

  // Control mode service
  auto reset_dofs_service_callback =
    [&](
      const std::shared_ptr<ControllerResetDofsSrvType::Request> request,
      std::shared_ptr<ControllerResetDofsSrvType::Response> response)
  {
    response->ok = true;

    if (
      (request->positions.size() != request->velocities.size()) ||
      (request->velocities.size() != request->accelerations.size()))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Reset dofs service call has different values size for positions %ld, velocities %ld, "
        "accelerations %ld",
        request->positions.size(), request->velocities.size(), request->accelerations.size());
      response->ok = false;
      return;
    }

    if ((request->positions.size() > 0) && (request->names.size() != request->positions.size()))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Reset dofs service call has names size %ld different than positions size %ld",
        request->names.size(), request->positions.size());
      response->ok = false;
      return;
    }

    std::vector<ResetDofsData> reset_flags_reset;
    reset_flags_reset.resize(
      dof_, {false, std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});

    // Here we read current reset dofs flags and clear it. This is done so we can add this new
    // request to the existing reset flags. This logic prevents this new request from overwriting
    // any previous request that hasn't been processed yet. The one assumption made here is that the
    // current reset flags are not going to be processed between the two calls here to read and
    // reset, which is a highly unlikely scenario. Even if it was, the behavior is fairly benign in
    // that the dofs in the previous request will be reset twice.
    auto reset_flags_local = *reset_dofs_flags_.readFromNonRT();
    reset_dofs_flags_.writeFromNonRT(reset_flags_reset);

    // add/update reset dofs flags from request
    for (size_t i = 0; i < request->names.size(); ++i)
    {
      auto it =
        std::find(command_axis_names_.begin(), command_axis_names_.end(), request->names[i]);
      if (it != command_axis_names_.end())
      {
        auto dist = std::distance(command_axis_names_.begin(), it);
        std::size_t cmd_itf_index =
          dist >= 0 ? static_cast<std::size_t>(dist)
                    : throw std::runtime_error("Joint not found in command joint names");
        double pos = (request->positions.size() != 0) ? request->positions[i]
                                                      : std::numeric_limits<double>::quiet_NaN();

        if (request->positions.size() != 0)
        {
          RCLCPP_INFO(get_node()->get_logger(), "Resetting dof '%s'", request->names[i].c_str());
        }
        else
        {
          RCLCPP_INFO(
            get_node()->get_logger(), "Resetting dof '%s' position to %f",
            request->names[i].c_str(), pos);
        }
        double vel = (request->velocities.size() != 0) ? request->velocities[i]
                                                       : std::numeric_limits<double>::quiet_NaN();
        double accel = (request->accelerations.size() != 0)
                         ? request->accelerations[i]
                         : std::numeric_limits<double>::quiet_NaN();
        reset_flags_local[cmd_itf_index] = {true, pos, vel, accel};
      }
      else
      {
        RCLCPP_WARN(
          get_node()->get_logger(), "Name '%s' is not command interface. Ignoring this entry.",
          request->names[i].c_str());
        response->ok = false;
      }
    }

    reset_dofs_flags_.writeFromNonRT(reset_flags_local);
  };

  reset_dofs_service_ = get_node()->create_service<ControllerResetDofsSrvType>(
    "~/reset_dofs", reset_dofs_service_callback);

  return CallbackReturn::SUCCESS;
}

void MultiTimeTrajectoryController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  last_reference_ = *msg;
  add_new_trajectory_msg(msg);
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "MAC activating...");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // parse remaining parameters
  default_tolerances_ = get_segment_tolerances(params_);

  // order all axes in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto dist = std::distance(allowed_interface_types_.begin(), it);
    std::size_t index =
      dist >= 0 ? static_cast<std::size_t>(dist)
                : throw std::runtime_error("Interface not found in allowed interface types");
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_axis_names_, interface, axis_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), axis_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto dist = std::distance(allowed_interface_types_.begin(), it);
    std::size_t index =
      dist >= 0 ? static_cast<std::size_t>(dist)
                : throw std::runtime_error("Interface not found in allowed interface types");
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.axes, interface, axis_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", dof_,
        interface.c_str(), axis_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<control_msgs::msg::MultiAxisTrajectory>());

  subscriber_is_active_ = true;

  // Initialize current state storage if hardware state has tracking offset
  // Handle restart of controller by reading from commands if those are not NaN (a controller was
  // running already)
  std::vector<control_msgs::msg::AxisTrajectoryPoint> state;
  state.resize(dof_);

  // Handle restart of controller by reading from commands if
  // those are not nan
  if (!read_state_from_command_interfaces(state))
  {
    // Initialize current state storage from hardware
    if (!read_state_from_hardware(state))
    {
      return CallbackReturn::ERROR;
    };
  }
  state_current_ = state;
  state_desired_ = state;
  last_commanded_state_ = state;
  last_commanded_time_ = rclcpp::Time();

  // The controller should start by holding position at the beginning of active state
  add_new_trajectory_msg(set_hold_position());
  rt_is_holding_.writeFromNonRT(true);

  // parse timeout parameter
  if (params_.cmd_timeout > 0.0)
  {
    if (params_.cmd_timeout > default_tolerances_.goal_time_tolerance)
    {
      cmd_timeout_ = params_.cmd_timeout;
    }
    else
    {
      // deactivate timeout
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Command timeout must be higher than goal_time tolerance (%f vs. %f)", params_.cmd_timeout,
        default_tolerances_.goal_time_tolerance);
      cmd_timeout_ = 0.0;
    }
  }
  else
  {
    cmd_timeout_ = 0.0;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "MAC successfully activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled during deactivate transition.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  for (size_t index = 0; index < dof_; ++index)
  {
    if (has_position_command_interface_)
    {
      axis_command_interface_[0][index].get().set_value(
        axis_command_interface_[0][index].get().get_value());
    }

    if (has_velocity_command_interface_)
    {
      axis_command_interface_[1][index].get().set_value(0.0);
    }

    if (has_acceleration_command_interface_)
    {
      axis_command_interface_[2][index].get().set_value(0.0);
    }

    // TODO(anyone): How to halt when using effort commands?
    if (has_effort_command_interface_)
    {
      axis_command_interface_[3][index].get().set_value(0.0);
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    axis_command_interface_[index].clear();
    axis_state_interface_[index].clear();
  }
  release_interfaces();

  subscriber_is_active_ = false;

  traj_external_point_ptr_.reset();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool MultiTimeTrajectoryController::reset()
{
  subscriber_is_active_ = false;
  axis_command_subscriber_.reset();

  for (const auto & pid : pids_)
  {
    if (pid)
    {
      pid->reset();
    }
  }

  traj_external_point_ptr_.reset();

  return true;
}

controller_interface::CallbackReturn MultiTimeTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  // TODO(karsten1987): what to do?

  return CallbackReturn::SUCCESS;
}

void MultiTimeTrajectoryController::publish_state(
  const rclcpp::Time & time, const bool first_sample, const std::vector<int> & segment_start)
{
  // TODO(henrygerardmoore): add any useful debug/state info to this publish
  // active state? goals and tolerances, internal state, etc.
  if (state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.references = state_desired_;
    state_publisher_->msg_.feedbacks = state_current_;
    state_publisher_->msg_.errors = state_error_;
    state_publisher_->msg_.using_odometry_feedback = params_.use_feedback;
    state_publisher_->msg_.trajectory_active = has_active_trajectory();
    state_publisher_->msg_.last_odom_feedback = last_odom_feedback_;
    state_publisher_->msg_.goal = *traj_external_point_ptr_->get_trajectory_msg();
    state_publisher_->msg_.last_reference = last_reference_;
    state_publisher_->msg_.first_sample_in_trajectory = first_sample;
    state_publisher_->msg_.trajectory_start_time = traj_external_point_ptr_->start_time();
    state_publisher_->msg_.time_before_trajectory =
      traj_external_point_ptr_->time_before_trajectory();
    state_publisher_->msg_.state_before_trajectory =
      traj_external_point_ptr_->state_before_trajectory();
    state_publisher_->msg_.state_after_interpolation =
      traj_external_point_ptr_->state_after_interp();
    state_publisher_->msg_.state_after_joint_limit =
      traj_external_point_ptr_->state_after_joint_limit();
    state_publisher_->msg_.segment_start = segment_start;

    if (read_commands_from_command_interfaces(command_current_))
    {
      state_publisher_->msg_.outputs = command_current_;
    }

    state_publisher_->unlockAndPublish();
  }

  if (splines_output_publisher_ && splines_output_publisher_->trylock())
  {
    splines_output_publisher_->msg_.header.stamp = state_publisher_->msg_.header.stamp;
    splines_output_publisher_->msg_.references = splines_state_;

    splines_output_publisher_->unlockAndPublish();
  }

  if (ruckig_input_publisher_ && ruckig_input_publisher_->trylock())
  {
    ruckig_input_publisher_->msg_.header.stamp = state_publisher_->msg_.header.stamp;
    ruckig_input_publisher_->msg_.references = ruckig_input_state_;

    ruckig_input_publisher_->unlockAndPublish();
  }

  if (ruckig_input_target_publisher_ && ruckig_input_target_publisher_->trylock())
  {
    ruckig_input_target_publisher_->msg_.header.stamp = state_publisher_->msg_.header.stamp;
    ruckig_input_target_publisher_->msg_.references = ruckig_state_;
    ruckig_input_target_publisher_->unlockAndPublish();
  }
}

void MultiTimeTrajectoryController::topic_callback(
  const std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> msg)
{
  if (!validate_trajectory_msg(*msg))
  {
    return;
  }
  // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
  // always replace old msg with new one for now
  if (subscriber_is_active_)
  {
    add_new_trajectory_msg(msg);
    rt_is_holding_.writeFromNonRT(false);
  }
};

rclcpp_action::GoalResponse MultiTimeTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiTimeTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // Enter hold current position mode
    add_new_trajectory_msg(set_hold_position());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiTimeTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // mark a pending goal
  rt_has_pending_goal_.writeFromNonRT(true);

  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<control_msgs::msg::MultiAxisTrajectory>(goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
    rt_is_holding_.writeFromNonRT(false);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->axis_names = params_.axes;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void MultiTimeTrajectoryController::compute_error(
  std::vector<TrajectoryPoint> & error, const std::vector<TrajectoryPoint> & current,
  const std::vector<TrajectoryPoint> & desired) const
{
  error.resize(current.size());
  for (std::size_t index = 0; index < current.size(); ++index)
  {
    // error defined as the difference between current and desired
    if (axis_angle_wraparound_[index])
    {
      // if desired, the shortest_angular_distance is calculated, i.e., the error is
      //  normalized between -pi<error<pi
      error[index].position =
        angles::shortest_angular_distance(current[index].position, desired[index].position);
    }
    else
    {
      error[index].position = desired[index].position - current[index].position;
    }
    if (
      has_velocity_state_interface_ &&
      (has_velocity_command_interface_ || has_effort_command_interface_))
    {
      error[index].velocity = desired[index].velocity - current[index].velocity;
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error[index].acceleration = desired[index].acceleration - current[index].acceleration;
    }
  }
}

bool isEmpty(std::vector<control_msgs::msg::AxisTrajectoryPoint> traj)
{
  bool position_empty = std::any_of(
    traj.begin(), traj.end(),
    [](control_msgs::msg::AxisTrajectoryPoint point) { return std::isnan(point.position); });
  bool velocity_empty = std::any_of(
    traj.begin(), traj.end(),
    [](control_msgs::msg::AxisTrajectoryPoint point) { return std::isnan(point.velocity); });
  bool acceleration_empty = std::any_of(
    traj.begin(), traj.end(),
    [](control_msgs::msg::AxisTrajectoryPoint point) { return std::isnan(point.acceleration); });
  return position_empty && velocity_empty && acceleration_empty;
}

void MultiTimeTrajectoryController::sort_to_local_axis_order(
  std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> trajectory_msg) const
{
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->axis_names, params_.axes);

  // this will create an empty AxisTrajectory for any unfilled axes, but that is handled in
  // `Trajectory::update`
  std::vector<control_msgs::msg::AxisTrajectory> to_return(dof_);
  std::size_t message_index = 0;
  for (auto const internal_index : mapping_vector)
  {
    if (!isEmpty(trajectory_msg->axis_trajectories[message_index].axis_points))
    {
      to_return[internal_index] = trajectory_msg->axis_trajectories[message_index];
    }
    ++message_index;
  }
  trajectory_msg->axis_trajectories = to_return;
}

bool MultiTimeTrajectoryController::validate_trajectory_msg(
  const control_msgs::msg::MultiAxisTrajectory & trajectories) const
{
  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectories.header.stamp);

  if (trajectories.axis_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  if (trajectories.axis_trajectories.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return false;
  }

  for (std::size_t axis_index = 0; axis_index < trajectories.axis_names.size(); ++axis_index)
  {
    auto const & trajectory = trajectories.axis_trajectories[axis_index];
    const std::string & incoming_axis_name = trajectories.axis_names[axis_index];
    // If partial axes goals are not allowed, goal should specify all controller axes

    // If the starting time it set to 0.0, it means the controller should start it now.
    // Otherwise we check if the trajectory ends before the current time,
    // in which case it can be ignored.
    if (trajectory_start_time.seconds() != 0.0)
    {
      auto const trajectory_end_time =
        trajectory_start_time + trajectory.axis_points.back().time_from_start;
      if (trajectory_end_time < get_node()->now())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Received trajectory for axis %zu with non-zero start time (%f) that ends in the past "
          "(%f)",
          axis_index, trajectory_start_time.seconds(), trajectory_end_time.seconds());
        return false;
      }
    }

    auto it = std::find(params_.axes.begin(), params_.axes.end(), incoming_axis_name);
    if (it == params_.axes.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming axis %s doesn't match the controller's axes.",
        incoming_axis_name.c_str());
      return false;
    }

    if (!params_.allow_nonzero_velocity_at_trajectory_end)
    {
      auto const final_vel = trajectory.axis_points.back().velocity;
      if (fabs(final_vel) > std::numeric_limits<float>::epsilon())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Velocity of last trajectory point of axis %s is not zero: %.15f",
          incoming_axis_name.c_str(), final_vel);
        return false;
      }
    }

    rclcpp::Duration previous_traj_time(0ms);
    for (size_t i = 0; i < trajectory.axis_points.size(); ++i)
    {
      if (
        (i > 0) &&
        (rclcpp::Duration(trajectory.axis_points[i].time_from_start) <= previous_traj_time))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Time between points %zu and %zu is not strictly increasing, it is %f and %f "
          "respectively",
          i - 1, i, previous_traj_time.seconds(),
          rclcpp::Duration(trajectory.axis_points[i].time_from_start).seconds());
        return false;
      }
      previous_traj_time = trajectory.axis_points[i].time_from_start;

      const auto & points = trajectory.axis_points;
      bool traj_has_position = !std::any_of(
        points.begin(), points.end(),
        [](control_msgs::msg::AxisTrajectoryPoint point) { return std::isnan(point.position); });
      bool traj_has_velocity = !std::any_of(
        points.begin(), points.end(),
        [](control_msgs::msg::AxisTrajectoryPoint point) { return std::isnan(point.velocity); });
      bool traj_has_acceleration = !std::any_of(
        points.begin(), points.end(), [](control_msgs::msg::AxisTrajectoryPoint point)
        { return std::isnan(point.acceleration); });

      if (!(traj_has_position || traj_has_velocity || traj_has_acceleration))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Empty trajectory for axis %s", incoming_axis_name.c_str());
        return false;
      }

      if (!params_.allow_integration_in_goal_trajectories && !traj_has_position)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "No position in trajectory for axis %s and integration of goal trajectories is disabled",
          incoming_axis_name.c_str());
        return false;
      }
    }
  }
  return true;
}

void MultiTimeTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void MultiTimeTrajectoryController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

std::shared_ptr<control_msgs::msg::MultiAxisTrajectory>
MultiTimeTrajectoryController::set_hold_position()
{
  // Command to stay at current position
  for (std::size_t i = 0; i < dof_; ++i)
  {
    hold_position_msg_ptr_->axis_trajectories[i].axis_points = {state_current_[i]};
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].velocity = 0;
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].acceleration = 0;
  }

  // set flag, otherwise tolerances will be checked with holding position too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

std::shared_ptr<control_msgs::msg::MultiAxisTrajectory>
MultiTimeTrajectoryController::set_success_trajectory_point()
{
  // set last command to be repeated at success, no matter if it has nonzero velocity or
  // acceleration

  for (std::size_t i = 0; i < dof_; ++i)
  {
    hold_position_msg_ptr_->axis_trajectories[i].axis_points = {
      traj_external_point_ptr_->get_trajectory_msg()->axis_trajectories[i].axis_points.back()};
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].time_from_start =
      rclcpp::Duration(0, 0);
  }

  // set flag, otherwise tolerances will be checked with success_trajectory_point too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

bool MultiTimeTrajectoryController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

bool MultiTimeTrajectoryController::has_active_trajectory() const
{
  return traj_external_point_ptr_ != nullptr && traj_external_point_ptr_->has_trajectory_msg();
}

void MultiTimeTrajectoryController::update_pids()
{
  for (size_t i = 0; i < dof_; ++i)
  {
    const auto & gains = params_.gains.axes_map.at(params_.axes[i]);
    if (pids_[i])
    {
      // update PIDs with gains from ROS parameters
      pids_[i]->setGains(gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);
    }
    else
    {
      // Init PIDs with gains from ROS parameters
      pids_[i] = std::make_shared<control_toolbox::Pid>(
        gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);
    }
    ff_velocity_scale_[i] = gains.ff_velocity_scale;
  }
}

void MultiTimeTrajectoryController::init_hold_position_msg()
{
  hold_position_msg_ptr_ = std::make_shared<control_msgs::msg::MultiAxisTrajectory>();
  hold_position_msg_ptr_->header.stamp =
    rclcpp::Time(0.0, 0.0, get_node()->get_clock()->get_clock_type());  // start immediately
  hold_position_msg_ptr_->axis_names = params_.axes;
  hold_position_msg_ptr_->axis_trajectories.resize(dof_);
  for (std::size_t i = 0; i < dof_; ++i)
  {
    hold_position_msg_ptr_->axis_trajectories[i].axis_points.resize(
      1, emptyTrajectoryPoint());  // a trivial msg only
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].velocity =
      std::numeric_limits<double>::quiet_NaN();
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].acceleration =
      std::numeric_limits<double>::quiet_NaN();
    hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].effort =
      std::numeric_limits<double>::quiet_NaN();
  }
  if (has_velocity_command_interface_ || has_acceleration_command_interface_)
  {
    // add velocity, so that trajectory sampling returns velocity points in any case
    for (std::size_t i = 0; i < dof_; ++i)
    {
      hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].velocity = 0;
    }
  }
  if (has_acceleration_command_interface_)
  {
    // add acceleration, so that trajectory sampling returns acceleration points in any case
    for (std::size_t i = 0; i < dof_; ++i)
    {
      hold_position_msg_ptr_->axis_trajectories[i].axis_points[0].acceleration = 0;
    }
  }
}

}  // namespace multi_time_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  multi_time_trajectory_controller::MultiTimeTrajectoryController,
  controller_interface::ControllerInterface)
