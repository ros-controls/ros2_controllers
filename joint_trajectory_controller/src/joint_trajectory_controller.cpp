// Copyright (c) 2021 ros2_control Development Team
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

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 0, 0)
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

namespace joint_trajectory_controller
{
JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(), dof_(0), num_cmd_joints_(0)
{
}

controller_interface::CallbackReturn JointTrajectoryController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
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
      /// initialize the URDF model and update the joint angles wraparound vector
      // Configure joint position error normalization (angle_wraparound)
      joints_angle_wraparound_.resize(params_.joints.size(), false);
      for (size_t i = 0; i < params_.joints.size(); ++i)
      {
        auto urdf_joint = model.getJoint(params_.joints[i]);
        if (urdf_joint && urdf_joint->type == urdf::Joint::CONTINUOUS)
        {
          RCLCPP_DEBUG(
            get_node()->get_logger(), "joint '%s' is of type continuous, use angle_wraparound.",
            params_.joints[i].c_str());
          joints_angle_wraparound_[i] = true;
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

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(num_cmd_joints_ * params_.command_interfaces.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
JointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::return_type JointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = this->get_node()->get_logger();
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(logger, params_);
    // update the PID gains
    // variable use_closed_loop_pid_adapter_ is updated in on_configure only
    if (use_closed_loop_pid_adapter_)
    {
      update_pids();
    }
  }

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new trajectory message has been received from Non-RT threads
  const auto current_trajectory_msg = current_trajectory_->get_trajectory_msg();
  auto new_external_msg = new_trajectory_msg_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (
    current_trajectory_msg != *new_external_msg &&
    (*(rt_has_pending_goal_.readFromRT()) && !active_goal) == false)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    current_trajectory_->update(*new_external_msg);
  }

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_state_interfaces(state_current_);

  // currently carrying out a trajectory
  if (has_active_trajectory())
  {
    bool first_sample = false;
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    // if sampling the first time, set the point before you sample
    if (!current_trajectory_->is_sampled_already())
    {
      first_sample = true;
      if (params_.interpolate_from_desired_state || params_.open_loop_control)
      {
        if (std::abs(last_commanded_time_.seconds()) < std::numeric_limits<float>::epsilon())
        {
          last_commanded_time_ = time;
        }
        current_trajectory_->set_point_before_trajectory_msg(
          last_commanded_time_, last_commanded_state_, joints_angle_wraparound_);
      }
      else
      {
        current_trajectory_->set_point_before_trajectory_msg(
          time, state_current_, joints_angle_wraparound_);
      }
      traj_time_ = time;
    }
    else
    {
      traj_time_ += period;
    }

    // Sample expected state from the trajectory
    current_trajectory_->sample(
      traj_time_, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    // Sample setpoint for next control cycle
    const bool valid_point = current_trajectory_->sample(
      traj_time_ + update_period_, interpolation_method_, command_next_, start_segment_itr,
      end_segment_itr, false);

    if (valid_point)
    {
      const rclcpp::Time traj_start = current_trajectory_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = traj_time_.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != current_trajectory_->end();
      auto active_tol = active_tolerances_.readFromRT();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (
        !before_last_point && *(rt_is_holding_.readFromRT()) == false && cmd_timeout_ > 0.0 &&
        time_difference > cmd_timeout_)
      {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index)
      {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if (
          (before_last_point || first_sample) && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, index, active_tol->state_tolerance[index], true /* show_errors */))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point && *(rt_is_holding_.readFromRT()) == false &&
          !check_state_tolerance_per_joint(
            state_error_, index, active_tol->goal_state_tolerance[index], false /* show_errors */))
        {
          outside_goal_tolerance = true;

          if (active_tol->goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_tolerance set it to aborted
            if (time_difference > active_tol->goal_time_tolerance)
            {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(
                state_error_, index, default_tolerances_.goal_state_tolerance[index],
                true /* show_errors */);
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time)
      {
        if (use_closed_loop_pid_adapter_)
        {
          // Update PIDs
          for (auto i = 0ul; i < num_cmd_joints_; ++i)
          {
            // If effort interface only, add desired effort as feed forward
            // If velocity interface, ignore desired effort
            size_t index_cmd_joint = map_cmd_to_joints_[i];
            tmp_command_[index_cmd_joint] =
              (command_next_.velocities[index_cmd_joint] * ff_velocity_scale_[i]) +
              (has_effort_command_interface_ ? command_next_.effort[index_cmd_joint] : 0.0) +
              pids_[i]->compute_command(
                state_error_.positions[index_cmd_joint], state_error_.velocities[index_cmd_joint],
                period);
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[0], command_next_.positions);
        }
        if (has_velocity_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[1], command_next_.velocities);
          }
        }
        if (has_acceleration_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[2], command_next_.accelerations);
        }
        if (has_effort_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          }
          else
          {
            // If position and effort command interfaces, only pass desired effort
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command and time used in open-loop control mode
        last_commanded_state_ = command_next_;
        last_commanded_time_ = time;
      }

      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
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

          RCLCPP_WARN(logger, "Aborted due to state tolerance violation");

          new_trajectory_msg_.reset();
          new_trajectory_msg_.initRT(set_hold_position());
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

            RCLCPP_INFO(logger, "Goal reached, success!");

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_success_trajectory_point());
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

            RCLCPP_WARN(logger, "%s", error_string.c_str());

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_hold_position());
          }
        }
      }
      else if (tolerance_violated_while_moving && *(rt_has_pending_goal_.readFromRT()) == false)
      {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(logger, "Holding position due to state tolerance violation");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      else if (
        !before_last_point && !within_goal_time && *(rt_has_pending_goal_.readFromRT()) == false)
      {
        RCLCPP_ERROR(logger, "Exceeded goal_time_tolerance: holding position...");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  publish_state(time, state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

void JointTrajectoryController::read_state_from_state_interfaces(JointTrajectoryPoint & state)
{
  auto logger = get_node()->get_logger();
  auto assign_point_from_state_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      const auto joint_state_interface_value_op = joint_interface[index].get().get_optional();
      if (!joint_state_interface_value_op.has_value())
      {
        RCLCPP_DEBUG(
          logger, "Unable to retrieve joint state interface value for joint at index %zu", index);
      }
      trajectory_point_interface[index] = joint_state_interface_value_op.value();
    }
  };
  auto assign_point_from_command_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    std::fill(
      trajectory_point_interface.begin(), trajectory_point_interface.end(),
      std::numeric_limits<double>::quiet_NaN());
    for (size_t index = 0; index < num_cmd_joints_; ++index)
    {
      const auto joint_command_interface_value_op = joint_interface[index].get().get_optional();
      if (!joint_command_interface_value_op.has_value())
      {
        RCLCPP_DEBUG(
          logger, "Unable to retrieve joint command interface value for joint at index %zu", index);
      }
      trajectory_point_interface[map_cmd_to_joints_[index]] =
        joint_command_interface_value_op.value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_state_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    assign_point_from_state_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    if (has_acceleration_state_interface_)
    {
      assign_point_from_state_interface(state.accelerations, joint_state_interface_[2]);
    }
    else
    {
      // Make empty so the property is ignored during interpolation
      state.accelerations.clear();
    }
  }
  else
  {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
  // No state interface for now, use command interface
  if (has_effort_command_interface_)
  {
    assign_point_from_command_interface(state.effort, joint_command_interface_[3]);
  }
}

bool JointTrajectoryController::read_state_from_command_interfaces(JointTrajectoryPoint & state)
{
  bool has_values = true;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < num_cmd_joints_; ++index)
    {
      const auto joint_interface_value_op = joint_interface[index].get().get_optional();
      if (!joint_interface_value_op.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(),
          "Unable to retrieve value of joint interface for joint at index %zu", index);
      }
      trajectory_point_interface[map_cmd_to_joints_[index]] = joint_interface_value_op.value();
    }
  };

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(), [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0]))
  {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  }
  else
  {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1]))
    {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
    }
    else
    {
      state.velocities.clear();
      has_values = false;
    }
  }
  else
  {
    state.velocities.clear();
  }
  // Acceleration is used only in combination with velocity
  if (has_acceleration_state_interface_)
  {
    if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2]))
    {
      assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
    }
    else
    {
      state.accelerations.clear();
      has_values = false;
    }
  }
  else
  {
    state.accelerations.clear();
  }

  // Effort state always comes from last command
  if (has_effort_command_interface_)
  {
    if (interface_has_values(joint_command_interface_[3]))
    {
      assign_point_from_interface(state.effort, joint_command_interface_[3]);
    }
    else
    {
      state.effort.clear();
      has_values = false;
    }
  }

  return has_values;
}

bool JointTrajectoryController::read_commands_from_command_interfaces(
  JointTrajectoryPoint & commands)
{
  bool has_values = true;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < num_cmd_joints_; ++index)
    {
      trajectory_point_interface[map_cmd_to_joints_[index]] =
        joint_interface[index].get().get_value();
    }
  };

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(), [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as command.
  if (has_position_command_interface_)
  {
    if (interface_has_values(joint_command_interface_[0]))
    {
      assign_point_from_interface(commands.positions, joint_command_interface_[0]);
    }
    else
    {
      commands.positions.clear();
      has_values = false;
    }
  }
  if (has_velocity_command_interface_)
  {
    if (interface_has_values(joint_command_interface_[1]))
    {
      assign_point_from_interface(commands.velocities, joint_command_interface_[1]);
    }
    else
    {
      commands.velocities.clear();
      has_values = false;
    }
  }
  if (has_acceleration_command_interface_)
  {
    if (interface_has_values(joint_command_interface_[2]))
    {
      assign_point_from_interface(commands.accelerations, joint_command_interface_[2]);
    }
    else
    {
      commands.accelerations.clear();
      has_values = false;
    }
  }
  if (has_effort_command_interface_)
  {
    if (interface_has_values(joint_command_interface_[3]))
    {
      assign_point_from_interface(commands.effort, joint_command_interface_[3]);
    }
    else
    {
      commands.effort.clear();
      has_values = false;
    }
  }

  return has_values;
}

void JointTrajectoryController::query_state_service(
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
  response->name = params_.joints;
  trajectory_msgs::msg::JointTrajectoryPoint state_requested = state_current_;
  if (has_active_trajectory())
  {
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    response->success = current_trajectory_->sample(
      static_cast<rclcpp::Time>(request->time), interpolation_method_, state_requested,
      start_segment_itr, end_segment_itr);
    // If the requested sample time precedes the trajectory finish time respond as failure
    if (response->success)
    {
      if (end_segment_itr == current_trajectory_->end())
      {
        RCLCPP_ERROR(logger, "Requested sample time precedes the current trajectory end time.");
        response->success = false;
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
  }
  response->position = state_requested.positions;
  response->velocity = state_requested.velocities;
  response->acceleration = state_requested.accelerations;
}

controller_interface::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // START DEPRECATE
  if (params_.open_loop_control)
  {
    RCLCPP_WARN(
      logger,
      "[deprecated] 'open_loop_control' parameter is deprecated. Instead, set the feedback gains "
      "to zero and use 'interpolate_from_desired_state' parameter");
  }
  // END DEPRECATE

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // get degrees of freedom
  dof_ = params_.joints.size();

  // TODO(destogl): why is this here? Add comment or move
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  num_cmd_joints_ = command_joint_names_.size();

  if (num_cmd_joints_ > dof_)
  {
    RCLCPP_ERROR(
      logger, "The 'command_joints' parameter must not exceed the size of the 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }
  else if (num_cmd_joints_ < dof_)
  {
    // create a map for the command joints
    map_cmd_to_joints_ = mapping(command_joint_names_, params_.joints);
    if (map_cmd_to_joints_.size() != num_cmd_joints_)
    {
      RCLCPP_ERROR(
        logger,
        "'command_joints' parameter must be a subset of 'joints' parameter, if their size is not "
        "equal.");
      return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < command_joint_names_.size(); i++)
    {
      RCLCPP_DEBUG(
        logger, "Command joint %lu: '%s' maps to joint %lu: '%s'.", i,
        command_joint_names_[i].c_str(), map_cmd_to_joints_[i],
        params_.joints.at(map_cmd_to_joints_[i]).c_str());
    }
  }
  else
  {
    // create a map for the command joints, trivial if the size is the same
    map_cmd_to_joints_.resize(num_cmd_joints_);
    std::iota(map_cmd_to_joints_.begin(), map_cmd_to_joints_.end(), 0);
  }

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // if there is only velocity or if there is effort command interface
  // then use also PID adapter
  use_closed_loop_pid_adapter_ =
    (has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
     !params_.open_loop_control) ||
    (has_effort_command_interface_ && params_.command_interfaces.size() == 1);

  tmp_command_.resize(dof_, 0.0);

  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(num_cmd_joints_);
    ff_velocity_scale_.resize(num_cmd_joints_);

    update_pids();
  }

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used. Still, for this is OK.
  joint_state_interface_.resize(allowed_interface_types_.size());

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

  // effort only or effort and position command interfaces require position and velocity state
  if (
    has_effort_command_interface_ &&
    (params_.command_interfaces.size() == 1 ||
     (params_.command_interfaces.size() == 2 && has_position_command_interface_)) &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'effort' command interface can only be used alone or with 'position' command interface "
      "if 'velocity' and 'position' state interfaces are present");
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
  default_tolerances_ = get_segment_tolerances(logger, params_);
  active_tolerances_.initRT(default_tolerances_);
  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());

  // prepare hold_position_msg
  init_hold_position_msg();

  // create subscriber and publishers
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryController::topic_callback, this, std::placeholders::_1));

  publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = params_.joints;
  state_publisher_->msg_.reference.positions.resize(dof_);
  state_publisher_->msg_.reference.velocities.resize(dof_);
  state_publisher_->msg_.reference.accelerations.resize(dof_);
  state_publisher_->msg_.feedback.positions.resize(dof_);
  state_publisher_->msg_.error.positions.resize(dof_);
  if (has_velocity_state_interface_)
  {
    state_publisher_->msg_.feedback.velocities.resize(dof_);
    state_publisher_->msg_.error.velocities.resize(dof_);
  }
  if (has_acceleration_state_interface_)
  {
    state_publisher_->msg_.feedback.accelerations.resize(dof_);
    state_publisher_->msg_.error.accelerations.resize(dof_);
  }
  if (has_position_command_interface_)
  {
    state_publisher_->msg_.output.positions.resize(dof_);
  }
  if (has_velocity_command_interface_)
  {
    state_publisher_->msg_.output.velocities.resize(dof_);
  }
  if (has_acceleration_command_interface_)
  {
    state_publisher_->msg_.output.accelerations.resize(dof_);
  }
  if (has_effort_command_interface_)
  {
    state_publisher_->msg_.output.effort.resize(dof_);
  }

  state_publisher_->unlock();

  // action server configuration
  if (params_.allow_partial_joints_goal)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&JointTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&JointTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&JointTrajectoryController::goal_accepted_callback, this, _1));

  resize_joint_trajectory_point(state_current_, dof_);
  resize_joint_trajectory_point_command(
    command_current_, dof_, std::numeric_limits<double>::quiet_NaN());
  resize_joint_trajectory_point(state_desired_, dof_);
  resize_joint_trajectory_point(state_error_, dof_);
  resize_joint_trajectory_point(
    last_commanded_state_, dof_, std::numeric_limits<double>::quiet_NaN());

  query_state_srv_ = get_node()->create_service<control_msgs::srv::QueryTrajectoryState>(
    std::string(get_node()->get_name()) + "/query_state",
    std::bind(&JointTrajectoryController::query_state_service, this, _1, _2));

  if (get_update_rate() == 0)
  {
    throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
  }
  update_period_ =
    rclcpp::Duration(0.0, static_cast<uint32_t>(1.0e9 / static_cast<double>(get_update_rate())));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // parse remaining parameters
  default_tolerances_ = get_segment_tolerances(logger, params_);

  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", num_cmd_joints_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(),
        joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  current_trajectory_ = std::make_shared<Trajectory>();
  new_trajectory_msg_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;

  // Handle restart of controller by reading from commands if those are not NaN (a controller was
  // running already)
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  // read from cmd joints only if all joints have command interface
  // otherwise it leaves the entries of joints without command interface NaN.
  // if no open_loop control, state_current_ is then used for `set_point_before_trajectory_msg`
  // and future trajectory sampling will always give NaN for these joints
  if (
    params_.set_last_command_interface_value_as_state_on_activation && dof_ == num_cmd_joints_ &&
    read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    last_commanded_state_ = state;
  }
  else
  {
    // Initialize current state storage from hardware
    read_state_from_state_interfaces(state_current_);
    read_state_from_state_interfaces(last_commanded_state_);
  }
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
        logger, "Command timeout must be higher than goal_time tolerance (%f vs. %f)",
        params_.cmd_timeout, default_tolerances_.goal_time_tolerance);
      cmd_timeout_ = 0.0;
    }
  }
  else
  {
    cmd_timeout_ = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  auto logger = get_node()->get_logger();
  if (active_goal)
  {
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled during deactivate transition.");
    active_goal->setAborted(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  for (size_t index = 0; index < num_cmd_joints_; ++index)
  {
    if (has_position_command_interface_)
    {
      const auto joint_position_value_op = joint_command_interface_[0][index].get().get_optional();
      if (!joint_position_value_op.has_value())
      {
        RCLCPP_WARN(logger, "Unable to retrieve joint position value");
        return controller_interface::CallbackReturn::SUCCESS;
      }
      if (!joint_command_interface_[0][index].get().set_value(joint_position_value_op.value()))
      {
        RCLCPP_WARN(
          logger, "Unable to set the joint position to value: %f",
          joint_position_value_op.value());
        return controller_interface::CallbackReturn::SUCCESS;
      }
    }

    if (has_velocity_command_interface_ && !joint_command_interface_[1][index].get().set_value(0.0))
    {
      RCLCPP_WARN(logger, "Error while setting joint velocity to value 0.0");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    if (
      has_acceleration_command_interface_ &&
      !joint_command_interface_[2][index].get().set_value(0.0))
    {
      RCLCPP_WARN(logger, "Error while setting joint acceleration to value 0.0");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // TODO(anyone): How to halt when using effort commands?
    if (has_effort_command_interface_ && !joint_command_interface_[3][index].get().set_value(0.0))
    {
      RCLCPP_WARN(logger, "Error while setting joint effort to value 0.0");
      return controller_interface::CallbackReturn::SUCCESS;
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  subscriber_is_active_ = false;

  current_trajectory_.reset();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool JointTrajectoryController::reset()
{
  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();

  for (const auto & pid : pids_)
  {
    if (pid)
    {
      pid->reset();
    }
  }

  current_trajectory_.reset();

  return true;
}

void JointTrajectoryController::publish_state(
  const rclcpp::Time & time, const JointTrajectoryPoint & desired_state,
  const JointTrajectoryPoint & current_state, const JointTrajectoryPoint & state_error)
{
  if (state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.reference.positions = desired_state.positions;
    state_publisher_->msg_.reference.velocities = desired_state.velocities;
    state_publisher_->msg_.reference.accelerations = desired_state.accelerations;
    state_publisher_->msg_.feedback.positions = current_state.positions;
    state_publisher_->msg_.error.positions = state_error.positions;
    if (has_velocity_state_interface_)
    {
      state_publisher_->msg_.feedback.velocities = current_state.velocities;
      state_publisher_->msg_.error.velocities = state_error.velocities;
    }
    if (has_acceleration_state_interface_)
    {
      state_publisher_->msg_.feedback.accelerations = current_state.accelerations;
      state_publisher_->msg_.error.accelerations = state_error.accelerations;
    }
    if (read_commands_from_command_interfaces(command_current_))
    {
      state_publisher_->msg_.output = command_current_;
    }

    state_publisher_->unlockAndPublish();
  }
}

void JointTrajectoryController::topic_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
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

rclcpp_action::GoalResponse JointTrajectoryController::goal_received_callback(
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

rclcpp_action::CancelResponse JointTrajectoryController::goal_cancelled_callback(
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

void JointTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // mark a pending goal
  rt_has_pending_goal_.writeFromNonRT(true);

  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
    rt_is_holding_.writeFromNonRT(false);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Update tolerances if specified in the goal
  auto logger = this->get_node()->get_logger();
  active_tolerances_.writeFromNonRT(get_segment_tolerances(
    logger, default_tolerances_, *(goal_handle->get_goal()), params_.joints));

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void JointTrajectoryController::compute_error_for_joint(
  JointTrajectoryPoint & error, const size_t index, const JointTrajectoryPoint & current,
  const JointTrajectoryPoint & desired) const
{
  // error defined as the difference between current and desired
  if (joints_angle_wraparound_[index])
  {
    // if desired, the shortest_angular_distance is calculated, i.e., the error is
    //  normalized between -pi<error<pi
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
  }
  else
  {
    error.positions[index] = desired.positions[index] - current.positions[index];
  }
  if (
    has_velocity_state_interface_ &&
    (has_velocity_command_interface_ || has_effort_command_interface_))
  {
    error.velocities[index] = desired.velocities[index] - current.velocities[index];
  }
  if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
  {
    error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
  }
}

void JointTrajectoryController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // joint names in the goal are a subset of existing joints, as checked in goal_callback
  // so if the size matches, the goal contains all controller joints
  if (dof_ == trajectory_msg->joint_names.size())
  {
    return;
  }

  trajectory_msg->joint_names.reserve(dof_);

  for (size_t index = 0; index < dof_; ++index)
  {
    {
      if (
        std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          params_.joints[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(params_.joints[index]);

      for (auto & it : trajectory_msg->points)
      {
        // Assume hold position with 0 velocity and acceleration for missing joints
        if (!it.positions.empty())
        {
          const auto position_command_value_op =
            joint_command_interface_[0][index].get().get_optional();
          if (!position_command_value_op.has_value())
          {
            RCLCPP_DEBUG(
              get_node()->get_logger(),
              "Unable to retrieve position command value of joint at index %zu", index);
          }
          if (has_position_command_interface_)
          {
            // copy last command if cmd interface exists
            it.positions.push_back(position_command_value_op.has_value());
          }
          else if (has_position_state_interface_)
          {
            // copy current state if state interface exists
            const auto position_state_value_op =
              joint_state_interface_[0][index].get().get_optional();
            if (!position_state_value_op.has_value())
            {
              RCLCPP_DEBUG(
                get_node()->get_logger(),
                "Unable to retrieve position state value of joint at index %zu", index);
            }
            it.positions.push_back(position_state_value_op.value());
          }
        }
        if (!it.velocities.empty())
        {
          it.velocities.push_back(0.0);
        }
        if (!it.accelerations.empty())
        {
          it.accelerations.push_back(0.0);
        }
        if (!it.effort.empty())
        {
          it.effort.push_back(0.0);
        }
      }
    }
  }
}

void JointTrajectoryController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, params_.joints);
  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & mapping) -> std::vector<double>
  {
    if (to_remap.empty())
    {
      return to_remap;
    }
    if (to_remap.size() != mapping.size())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
      return to_remap;
    }
    static std::vector<double> output(dof_, 0.0);
    // Only resize if necessary since it's an expensive operation
    if (output.size() != mapping.size())
    {
      output.resize(mapping.size(), 0.0);
    }
    for (size_t index = 0; index < mapping.size(); ++index)
    {
      auto map_index = mapping[index];
      output[map_index] = to_remap[index];
    }
    return output;
  };

  for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
  {
    trajectory_msg->points[index].positions =
      remap(trajectory_msg->points[index].positions, mapping_vector);

    trajectory_msg->points[index].velocities =
      remap(trajectory_msg->points[index].velocities, mapping_vector);

    trajectory_msg->points[index].accelerations =
      remap(trajectory_msg->points[index].accelerations, mapping_vector);

    trajectory_msg->points[index].effort =
      remap(trajectory_msg->points[index].effort, mapping_vector);
  }
}

bool JointTrajectoryController::validate_trajectory_point_field(
  size_t joint_names_size, const std::vector<double> & vector_field,
  const std::string & string_for_vector_field, size_t i, bool allow_empty) const
{
  if (allow_empty && vector_field.empty())
  {
    return true;
  }
  if (joint_names_size != vector_field.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatch between joint_names size (%zu) and %s (%zu) at point #%zu.", joint_names_size,
      string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

bool JointTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // CHECK: Partial joint goals
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!params_.allow_partial_joints_goal)
  {
    if (trajectory.joint_names.size() != dof_)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  // CHECK: if joint names are provided
  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  // CHECK: if provided trajectory has points
  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return false;
  }

  // CHECK: If joint names are matching the joints defined for the controller
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(params_.joints.begin(), params_.joints.end(), incoming_joint_name);
    if (it == params_.joints.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  // CHECK: if trajectory ends with non-zero velocity (when option is disabled)
  if (!params_.allow_nonzero_velocity_at_trajectory_end)
  {
    for (size_t i = 0; i < trajectory.points.back().velocities.size(); ++i)
    {
      if (fabs(trajectory.points.back().velocities.at(i)) > std::numeric_limits<float>::epsilon())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Velocity of last trajectory point of joint %s is not zero: %.15f",
          trajectory.joint_names.at(i).c_str(), trajectory.points.back().velocities.at(i));
        return false;
      }
    }
  }

  // CHECK: if trajectory end time is in the past (if start time defined)
  const rclcpp::Time trajectory_start_time = trajectory.header.stamp;
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto const trajectory_end_time =
      trajectory_start_time + trajectory.points.back().time_from_start;
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non-zero start time (%f) that ends in the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    // CHECK: if time of points in the trajectory is monotonous
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f "
        "respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;

    // CHECK: if all required data are provided in the trajectory
    // This currently supports only position, velocity and acceleration inputs
    if (params_.allow_integration_in_goal_trajectories)
    {
      if (
        points[i].positions.empty() && points[i].velocities.empty() &&
        points[i].accelerations.empty())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "The given trajectory has no position, velocity, or acceleration points.");
        return false;
      }
      const bool position_error =
        !points[i].positions.empty() &&
        !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false);
      const bool velocity_error =
        !points[i].velocities.empty() &&
        !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, false);
      const bool acceleration_error =
        !points[i].accelerations.empty() &&
        !validate_trajectory_point_field(
          joint_count, points[i].accelerations, "accelerations", i, false);
      if (position_error || velocity_error || acceleration_error)
      {
        return false;
      }
    }
    else if (
      !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].accelerations, "accelerations", i, true))
    {
      return false;
    }
    // reject effort entries
    if (!has_effort_command_interface_ && !points[i].effort.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Trajectories with effort fields are only supported for "
        "controllers using the 'effort' command interface.");
      return false;
    }
  }
  return true;
}

void JointTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  new_trajectory_msg_.writeFromNonRT(traj_msg);
}

void JointTrajectoryController::preempt_active_goal()
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

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::set_hold_position()
{
  // Command to stay at current position
  hold_position_msg_ptr_->points[0].positions = state_current_.positions;

  // set flag, otherwise tolerances will be checked with holding position too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::set_success_trajectory_point()
{
  // set last command to be repeated at success, no matter if it has nonzero velocity or
  // acceleration
  hold_position_msg_ptr_->points[0] = current_trajectory_->get_trajectory_msg()->points.back();
  hold_position_msg_ptr_->points[0].time_from_start = rclcpp::Duration(0, 0);

  // set flag, otherwise tolerances will be checked with success_trajectory_point too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

bool JointTrajectoryController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

void JointTrajectoryController::resize_joint_trajectory_point(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size, double value)
{
  point.positions.resize(size, value);
  if (has_velocity_state_interface_)
  {
    point.velocities.resize(size, value);
  }
  if (has_acceleration_state_interface_)
  {
    point.accelerations.resize(size, value);
  }
  point.effort.resize(size, 0.0);
}

void JointTrajectoryController::resize_joint_trajectory_point_command(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size, double value)
{
  if (has_position_command_interface_)
  {
    point.positions.resize(size, value);
  }
  if (has_velocity_command_interface_)
  {
    point.velocities.resize(size, value);
  }
  if (has_acceleration_command_interface_)
  {
    point.accelerations.resize(size, value);
  }
  if (has_effort_command_interface_)
  {
    point.effort.resize(size, value);
  }
}

bool JointTrajectoryController::has_active_trajectory() const
{
  return current_trajectory_ != nullptr && current_trajectory_->has_trajectory_msg();
}

void JointTrajectoryController::update_pids()
{
  for (size_t i = 0; i < num_cmd_joints_; ++i)
  {
    const auto & gains = params_.gains.joints_map.at(params_.joints.at(map_cmd_to_joints_[i]));
    if (pids_[i])
    {
      // update PIDs with gains from ROS parameters
      pids_[i]->set_gains(gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);
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

void JointTrajectoryController::init_hold_position_msg()
{
  hold_position_msg_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  hold_position_msg_ptr_->header.stamp =
    rclcpp::Time(0.0, 0.0, get_node()->get_clock()->get_clock_type());  // start immediately
  hold_position_msg_ptr_->joint_names = params_.joints;
  hold_position_msg_ptr_->points.resize(1);  // a trivial msg only
  hold_position_msg_ptr_->points[0].velocities.clear();
  hold_position_msg_ptr_->points[0].accelerations.clear();
  hold_position_msg_ptr_->points[0].effort.clear();
  if (has_velocity_command_interface_ || has_acceleration_command_interface_)
  {
    // add velocity, so that trajectory sampling returns velocity points in any case
    hold_position_msg_ptr_->points[0].velocities.resize(dof_, 0.0);
  }
  if (has_acceleration_command_interface_)
  {
    // add velocity, so that trajectory sampling returns acceleration points in any case
    hold_position_msg_ptr_->points[0].accelerations.resize(dof_, 0.0);
  }
  if (has_effort_command_interface_)
  {
    hold_position_msg_ptr_->points[0].effort.resize(dof_, 0.0);
  }
}

}  // namespace joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
