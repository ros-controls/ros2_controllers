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

#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"

namespace joint_trajectory_controller
{
JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(), dof_(0)
{
}

controller_interface::CallbackReturn JointTrajectoryController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    // Set interpolation method from string parameter
    interpolation_method_ = interpolation_methods::from_string(params_.interpolation_method);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryController::command_interface_configuration() const
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
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  auto compute_error_for_joint = [&](
                                   JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired)
  {
    // error defined as the difference between current and desired
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    if (has_velocity_state_interface_ && has_velocity_command_interface_)
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&](auto & joint_interface, const std::vector<double> & trajectory_point_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_hardware(state_current_);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
  {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      first_sample = true;
      if (params_.open_loop_control)
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
      }
      else
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
      }
    }

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
      (*traj_point_active_ptr_)
        ->sample(time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    if (valid_point)
    {
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      double time_difference = 0.0;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index)
      {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        if (
          (before_last_point || first_sample) &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.state_tolerance[index], false))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_tolerance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            time_difference = get_node()->now().seconds() - traj_end.seconds();

            if (time_difference > default_tolerances_.goal_time_tolerance)
            {
              within_goal_time = false;
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
          for (auto i = 0ul; i < dof_; ++i)
          {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                              pids_[i]->computeCommand(
                                state_desired_.positions[i] - state_current_.positions[i],
                                state_desired_.velocities[i] - state_current_.velocities[i],
                                (uint64_t)period.nanoseconds());
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
        }
        if (has_velocity_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
          }
        }
        if (has_acceleration_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
        }
        if (has_effort_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
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
          set_hold_position();
          auto result = std::make_shared<FollowJTrajAction::Result>();

          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

          // check goal tolerance
        }
        else if (!before_last_point)
        {
          if (!outside_goal_tolerance)
          {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
          }
          else if (!within_goal_time)
          {
            set_hold_position();
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            RCLCPP_WARN(
              get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
              time_difference);
          }
          // else, run another cycle while waiting for outside_goal_tolerance
          // to be satisfied or violated within the goal_time_tolerance
        }
      }
      else if (tolerance_violated_while_moving)
      {
        set_hold_position();
        RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");
      }
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

void JointTrajectoryController::read_state_from_hardware(JointTrajectoryPoint & state)
{
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    trajectory_point_interface.resize(dof_);
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  state.positions.clear();
  state.velocities.clear();
  state.accelerations.clear();

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    if (has_acceleration_state_interface_)
    {
      assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
    }
  }
}

bool JointTrajectoryController::read_state_from_command_interfaces(JointTrajectoryPoint & state)
{
  bool has_values = true;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(),
             [](const auto & interface)
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

  return has_values;
}

controller_interface::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // Check if the DoF has changed
  if ((dof_ > 0) && (params_.joints.size() != dof_))
  {
    RCLCPP_ERROR(
      logger,
      "The JointTrajectoryController does not support restarting with a different number of DOF");
    // TODO(andyz): update vector lengths if num. joints did change and re-initialize them so we
    // can continue
    return CallbackReturn::FAILURE;
  }

  dof_ = params_.joints.size();

  // TODO(destogl): why is this here? Add comment or move
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  if (params_.joints.empty())
  {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  else if (command_joint_names_.size() != params_.joints.size())
  {
    RCLCPP_ERROR(
      logger, "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  //  // Specialized, child controllers set interfaces before calling configure function.
  //  if (command_interface_types_.empty())
  //  {
  //   command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
  //  }

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
    (has_velocity_command_interface_ && params_.command_interfaces.size() == 1) ||
    has_effort_command_interface_;

  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(dof_);
    ff_velocity_scale_.resize(dof_);
    tmp_command_.resize(dof_, 0.0);

    // Init PID gains from ROS parameters
    for (size_t i = 0; i < dof_; ++i)
    {
      const auto & gains = params_.gains.joints_map.at(params_.joints[i]);
      pids_[i] = std::make_shared<control_toolbox::Pid>(
        gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);

      // TODO(destogl): remove this in ROS2 Iron
      // Check deprecated style for "ff_velocity_scale" parameter definition.
      if (gains.ff_velocity_scale == 0.0)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "'ff_velocity_scale' parameters is not defined under 'gains.<joint_name>.' structure. "
          "Maybe you are using deprecated format 'ff_velocity_scale/<joint_name>'!");

        ff_velocity_scale_[i] = auto_declare<double>("ff_velocity_scale/" + params_.joints[i], 0.0);
      }
      else
      {
        ff_velocity_scale_[i] = gains.ff_velocity_scale;
      }
    }
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

  default_tolerances_ = get_segment_tolerances(params_);

  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryController::topic_callback, this, std::placeholders::_1));

  // State publisher
  RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", params_.state_publish_rate);
  if (params_.state_publish_rate > 0.0)
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / params_.state_publish_rate);
  }
  else
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ =
    get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = params_.joints;
  state_publisher_->msg_.desired.positions.resize(dof_);
  state_publisher_->msg_.desired.velocities.resize(dof_);
  state_publisher_->msg_.desired.accelerations.resize(dof_);
  state_publisher_->msg_.actual.positions.resize(dof_);
  state_publisher_->msg_.error.positions.resize(dof_);
  if (has_velocity_state_interface_)
  {
    state_publisher_->msg_.actual.velocities.resize(dof_);
    state_publisher_->msg_.error.velocities.resize(dof_);
  }
  if (has_acceleration_state_interface_)
  {
    state_publisher_->msg_.actual.accelerations.resize(dof_);
    state_publisher_->msg_.error.accelerations.resize(dof_);
  }
  state_publisher_->unlock();

  last_state_publish_time_ = get_node()->now();

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
  resize_joint_trajectory_point(state_desired_, dof_);
  resize_joint_trajectory_point(state_error_, dof_);
  resize_joint_trajectory_point(last_commanded_state_, dof_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", dof_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(joint_state_interface_[0].size());
  for (size_t index = 0; index < joint_state_interface_[0].size(); ++index)
  {
    traj_msg_home_ptr_->points[0].positions[index] =
      joint_state_interface_[0][index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_home_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;
  last_state_publish_time_ = get_node()->now();

  // Initialize current state storage if hardware state has tracking offset
  read_state_from_hardware(state_current_);
  read_state_from_hardware(state_desired_);
  read_state_from_hardware(last_commanded_state_);
  // Handle restart of controller by reading from commands if
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    state_desired_ = state;
    last_commanded_state_ = state;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // TODO(anyone): How to halt when using effort commands?
  for (size_t index = 0; index < dof_; ++index)
  {
    if (has_position_command_interface_)
    {
      joint_command_interface_[0][index].get().set_value(
        joint_command_interface_[0][index].get().get_value());
    }

    if (has_velocity_command_interface_)
    {
      joint_command_interface_[1][index].get().set_value(0.0);
    }

    if (has_effort_command_interface_)
    {
      joint_command_interface_[3][index].get().set_value(0.0);
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

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

  // iterator has no default value
  // prev_traj_point_ptr_;
  traj_point_active_ptr_ = nullptr;
  traj_external_point_ptr_.reset();
  traj_home_point_ptr_.reset();
  traj_msg_home_ptr_.reset();

  return true;
}

controller_interface::CallbackReturn JointTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  // TODO(karsten1987): what to do?

  return CallbackReturn::SUCCESS;
}

void JointTrajectoryController::publish_state(
  const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
  const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0)
  {
    return;
  }

  if (get_node()->now() < (last_state_publish_time_ + state_publisher_period_))
  {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    last_state_publish_time_ = get_node()->now();
    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    state_publisher_->msg_.desired.positions = desired_state.positions;
    state_publisher_->msg_.desired.velocities = desired_state.velocities;
    state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
    state_publisher_->msg_.actual.positions = current_state.positions;
    state_publisher_->msg_.error.positions = state_error.positions;
    if (has_velocity_state_interface_)
    {
      state_publisher_->msg_.actual.velocities = current_state.velocities;
      state_publisher_->msg_.error.velocities = state_error.velocities;
    }
    if (has_acceleration_state_interface_)
    {
      state_publisher_->msg_.actual.accelerations = current_state.accelerations;
      state_publisher_->msg_.error.accelerations = state_error.accelerations;
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
  }
};

rclcpp_action::GoalResponse JointTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
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
    // Controller uptime
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
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
          if (
            has_position_command_interface_ &&
            !std::isnan(joint_command_interface_[0][index].get().get_value()))
          {
            // copy last command if cmd interface exists
            it.positions.push_back(joint_command_interface_[0][index].get().get_value());
          }
          else if (has_position_state_interface_)
          {
            // copy current state if state interface exists
            it.positions.push_back(joint_state_interface_[0][index].get().get_value());
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
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg)
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
    std::vector<double> output;
    output.resize(mapping.size(), 0.0);
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
      get_node()->get_logger(), "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.",
      joint_names_size, string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

bool JointTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
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

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto trajectory_end_time = trajectory_start_time;
    for (const auto & p : trajectory.points)
    {
      trajectory_end_time += p.time_from_start;
    }
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

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

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    // This currently supports only position, velocity and acceleration inputs
    if (params_.allow_integration_in_goal_trajectories)
    {
      const bool all_empty = points[i].positions.empty() && points[i].velocities.empty() &&
                             points[i].accelerations.empty();
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
      if (all_empty || position_error || velocity_error || acceleration_error)
      {
        return false;
      }
    }
    else if (
      !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].accelerations, "accelerations", i, true) ||
      !validate_trajectory_point_field(joint_count, points[i].effort, "effort", i, true))
    {
      return false;
    }
  }
  return true;
}

void JointTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void JointTrajectoryController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    set_hold_position();
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

void JointTrajectoryController::set_hold_position()
{
  trajectory_msgs::msg::JointTrajectory empty_msg;
  empty_msg.header.stamp = rclcpp::Time(0);

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg);
  add_new_trajectory_msg(traj_msg);
}

bool JointTrajectoryController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

void JointTrajectoryController::resize_joint_trajectory_point(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
{
  point.positions.resize(size, 0.0);
  if (has_velocity_state_interface_)
  {
    point.velocities.resize(size, 0.0);
  }
  if (has_acceleration_state_interface_)
  {
    point.accelerations.resize(size, 0.0);
  }
}

}  // namespace joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
