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
: controller_interface::ControllerInterface(), joint_names_({})
{
}

CallbackReturn JointTrajectoryController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::vector<std::string>>("joints", joint_names_);
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    auto_declare<double>("state_publish_rate", 50.0);
    auto_declare<double>("action_monitor_rate", 20.0);
    auto_declare<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
    auto_declare<bool>("open_loop_control", open_loop_control_);
    auto_declare<double>("constraints.stopped_velocity_tolerance", 0.01);
    auto_declare<double>("constraints.goal_time", 0.0);
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
  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
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
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::return_type JointTrajectoryController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  auto compute_error_for_joint = [&](
                                   JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired) {
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
    traj_external_point_ptr_->update(*new_external_msg);
  }

  JointTrajectoryPoint state_current, state_desired, state_error;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(state_current, joint_num);

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&, joint_num](auto & joint_inteface, const std::vector<double> & trajectory_point_interface) {
      for (size_t index = 0; index < joint_num; ++index)
      {
        joint_inteface[index].get().set_value(trajectory_point_interface[index]);
      }
    };

  // current state update
  state_current.time_from_start.set__sec(0);
  read_state_from_hardware(state_current);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
  {
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      if (open_loop_control_)
      {
        (*traj_point_active_ptr_)
          ->set_point_before_trajectory_msg(node_->now(), last_commanded_state_);
      }
      else
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(node_->now(), state_current);
      }
    }
    resize_joint_trajectory_point(state_error, joint_num);

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    // TODO(anyone): this is kind-of open-loop concept? I am right?
    const bool valid_point =
      (*traj_point_active_ptr_)
        ->sample(node_->now(), state_desired, start_segment_itr, end_segment_itr);

    if (valid_point)
    {
      bool abort = false;
      bool outside_goal_tolerance = false;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      // set values for next hardware write()
      if (has_position_command_interface_)
      {
        assign_interface_from_point(joint_command_interface_[0], state_desired.positions);
      }
      if (has_velocity_command_interface_)
      {
        assign_interface_from_point(joint_command_interface_[1], state_desired.velocities);
      }
      if (has_acceleration_command_interface_)
      {
        assign_interface_from_point(joint_command_interface_[2], state_desired.accelerations);
      }
      // TODO(anyone): Add here "if using_closed_loop_hw_interface_adapter" (see ROS1) - #171
      //       if (check_if_interface_type_exist(
      //           command_interface_types_, hardware_interface::HW_IF_EFFORT)) {
      //         assign_interface_from_point(joint_command_interface_[3], state_desired.effort);
      //       }

      for (size_t index = 0; index < joint_num; ++index)
      {
        compute_error_for_joint(state_error, index, state_current, state_desired);

        if (
          before_last_point &&
          !check_state_tolerance_per_joint(
            state_error, index, default_tolerances_.state_tolerance[index], false))
        {
          abort = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point &&
          !check_state_tolerance_per_joint(
            state_error, index, default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;
        }
      }

      // store command as state when hardware state has tracking offset
      last_commanded_state_ = state_desired;

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = node_->now();
        feedback->joint_names = joint_names_;

        feedback->actual = state_current;
        feedback->desired = state_desired;
        feedback->error = state_error;
        active_goal->setFeedback(feedback);

        // check abort
        if (abort || outside_goal_tolerance)
        {
          auto result = std::make_shared<FollowJTrajAction::Result>();

          if (abort)
          {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to state tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          }
          else if (outside_goal_tolerance)
          {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to goal tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
          }
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

            RCLCPP_INFO(node_->get_logger(), "Goal reached, success!");
          }
          else if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_toleralance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            const double difference = node_->now().seconds() - traj_end.seconds();
            if (difference > default_tolerances_.goal_time_tolerance)
            {
              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
              active_goal->setAborted(result);
              // TODO(matthew-reynolds): Need a lock-free write here
              // See https://github.com/ros-controls/ros2_controllers/issues/168
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              RCLCPP_WARN(
                node_->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
                difference);
            }
          }
        }
      }
    }
  }

  publish_state(state_desired, state_current, state_error);
  return controller_interface::return_type::OK;
}

void JointTrajectoryController::read_state_from_hardware(JointTrajectoryPoint & state)
{
  const auto joint_num = joint_names_.size();
  auto assign_point_from_interface =
    [&, joint_num](std::vector<double> & trajectory_point_interface, const auto & joint_inteface) {
      for (size_t index = 0; index < joint_num; ++index)
      {
        trajectory_point_interface[index] = joint_inteface[index].get().get_value();
      }
    };

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
}

bool JointTrajectoryController::read_state_from_command_interfaces(JointTrajectoryPoint & state)
{
  bool has_values = true;

  const auto joint_num = joint_names_.size();
  auto assign_point_from_interface =
    [&, joint_num](std::vector<double> & trajectory_point_interface, const auto & joint_inteface) {
      for (size_t index = 0; index < joint_num; ++index)
      {
        trajectory_point_interface[index] = joint_inteface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface) {
    return std::find_if(joint_interface.begin(), joint_interface.end(), [](const auto & interface) {
             return std::isnan(interface.get().get_value());
           }) == joint_interface.end();
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

CallbackReturn JointTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();

  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  if (joint_names_.empty())
  {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  // Specialized, child controllers set interfaces before calling configure function.
  if (command_interface_types_.empty())
  {
    command_interface_types_ = node_->get_parameter("command_interfaces").as_string_array();
  }

  if (command_interface_types_.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end())
    {
      RCLCPP_ERROR(logger, "Command interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  // Check if command interfaces combination is valid. Valid combinations are:
  // 1. effort
  // 2. velocity
  // 2. position [velocity, [acceleration]]

  // effort can't be combined with other interfaces
  if (contains_interface_type(command_interface_types_, hardware_interface::HW_IF_EFFORT))
  {
    if (command_interface_types_.size() == 1)
    {
      // TODO(anyone): This flag is not used for now
      // There should be PID-approach used as in ROS1:
      // https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/hardware_interface_adapter.h#L283
      use_closed_loop_pid_adapter = true;
      // TODO(anyone): remove the next two lines when implemented
      RCLCPP_ERROR(logger, "using 'effort' command interface alone is not yet implemented yet.");
      return CallbackReturn::FAILURE;
    }
    else
    {
      RCLCPP_ERROR(logger, "'effort' command interface has to be used alone.");
      return CallbackReturn::FAILURE;
    }
  }

  if (contains_interface_type(command_interface_types_, hardware_interface::HW_IF_POSITION))
  {
    has_position_command_interface_ = true;
  }
  if (contains_interface_type(command_interface_types_, hardware_interface::HW_IF_VELOCITY))
  {
    has_velocity_command_interface_ = true;
  }
  if (contains_interface_type(command_interface_types_, hardware_interface::HW_IF_ACCELERATION))
  {
    has_acceleration_command_interface_ = true;
  }

  if (has_velocity_command_interface_)
  {
    // if there is only velocity then use also PID adapter
    if (command_interface_types_.size() == 1)
    {
      use_closed_loop_pid_adapter = true;
      // TODO(anyone): remove this when implemented
      RCLCPP_ERROR(logger, "using 'velocity' command interface alone is not yet implemented yet.");
      return CallbackReturn::FAILURE;
      // if velocity interface can be used without position if multiple defined
    }
    else if (!has_position_command_interface_)
    {
      RCLCPP_ERROR(
        logger,
        "'velocity' command interface can be used either alone or 'position' "
        "interface has to be present.");
      return CallbackReturn::FAILURE;
    }
    // invalid: acceleration is defined and no velocity
  }
  else if (has_acceleration_command_interface_)
  {
    RCLCPP_ERROR(
      logger,
      "'acceleration' command interface can only be used if 'velocity' and "
      "'position' interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // Read always state interfaces from the parameter because they can be used
  // independently from the controller's type.
  // Specialized, child controllers should set its default value.
  state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();

  if (state_interface_types_.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  if (contains_interface_type(state_interface_types_, hardware_interface::HW_IF_EFFORT))
  {
    RCLCPP_ERROR(logger, "State interface type 'effort' not allowed!");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used. Still, for this is OK.
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : state_interface_types_)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end())
    {
      RCLCPP_ERROR(logger, "State interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  if (contains_interface_type(state_interface_types_, hardware_interface::HW_IF_VELOCITY))
  {
    has_velocity_state_interface_ = true;
  }
  if (contains_interface_type(state_interface_types_, hardware_interface::HW_IF_ACCELERATION))
  {
    has_acceleration_state_interface_ = true;
  }

  if (has_velocity_state_interface_)
  {
    if (!contains_interface_type(state_interface_types_, hardware_interface::HW_IF_POSITION))
    {
      RCLCPP_ERROR(
        logger,
        "'velocity' state interface cannot be used if 'position' interface "
        "is missing.");
      return CallbackReturn::FAILURE;
    }
  }
  else if (has_acceleration_state_interface_)
  {
    RCLCPP_ERROR(
      logger,
      "'acceleration' state interface cannot be used if 'position' and 'velocity' "
      "interfaces are not present.");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types) {
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
    logger, "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
    get_interface_list(state_interface_types_).c_str());

  default_tolerances_ = get_segment_tolerances(*node_, joint_names_);

  // Read parameters customizing controller for special cases
  open_loop_control_ = node_->get_parameter("open_loop_control").get_value<bool>();

  // subscriber callback
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) -> void {
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

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // TODO(karsten1987): no lifecycle for subscriber yet
  // joint_command_subscriber_->on_activate();

  // State publisher
  const double state_publish_rate = node_->get_parameter("state_publish_rate").get_value<double>();
  RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", state_publish_rate);
  if (state_publish_rate > 0.0)
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);
  }
  else
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ = node_->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  const auto n_joints = joint_names_.size();

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.desired.positions.resize(n_joints);
  state_publisher_->msg_.desired.velocities.resize(n_joints);
  state_publisher_->msg_.desired.accelerations.resize(n_joints);
  state_publisher_->msg_.actual.positions.resize(n_joints);
  state_publisher_->msg_.error.positions.resize(n_joints);
  if (has_velocity_state_interface_)
  {
    state_publisher_->msg_.actual.velocities.resize(n_joints);
    state_publisher_->msg_.error.velocities.resize(n_joints);
  }
  if (has_acceleration_state_interface_)
  {
    state_publisher_->msg_.actual.accelerations.resize(n_joints);
    state_publisher_->msg_.error.accelerations.resize(n_joints);
  }
  state_publisher_->unlock();

  last_state_publish_time_ = node_->now();

  // action server configuration
  allow_partial_joints_goal_ = node_->get_parameter("allow_partial_joints_goal").get_value<bool>();
  if (allow_partial_joints_goal_)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  const double action_monitor_rate =
    node_->get_parameter("action_monitor_rate").get_value<double>();

  RCLCPP_INFO(logger, "Action status changes will be monitored at %.2f Hz.", action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    node_->get_node_base_interface(), node_->get_node_clock_interface(),
    node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
    std::string(node_->get_name()) + "/follow_joint_trajectory",
    std::bind(&JointTrajectoryController::goal_callback, this, _1, _2),
    std::bind(&JointTrajectoryController::cancel_callback, this, _1),
    std::bind(&JointTrajectoryController::feedback_setup_callback, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  // order all joints in the storage
  for (const auto & interface : command_interface_types_)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : state_interface_types_)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
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
  last_state_publish_time_ = node_->now();

  // Initialize current state storage if hardware state has tracking offset
  resize_joint_trajectory_point(last_commanded_state_, joint_names_.size());
  read_state_from_hardware(last_commanded_state_);
  // Handle restart of controller by reading last_commanded_state_ from commands is
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, joint_names_.size());
  if (read_state_from_command_interfaces(state))
  {
    last_commanded_state_ = state;
  }

  // TODO(karsten1987): activate subscriptions of subscriber
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // TODO(anyone): How to halt when using effort commands?
  for (size_t index = 0; index < joint_names_.size(); ++index)
  {
    joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[0][index].get().get_value());
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

CallbackReturn JointTrajectoryController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryController::on_error(const rclcpp_lifecycle::State &)
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

  // iterator has no default value
  // prev_traj_point_ptr_;
  traj_point_active_ptr_ = nullptr;
  traj_external_point_ptr_.reset();
  traj_home_point_ptr_.reset();
  traj_msg_home_ptr_.reset();

  return true;
}

CallbackReturn JointTrajectoryController::on_shutdown(const rclcpp_lifecycle::State &)
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

  if (node_->now() < (last_state_publish_time_ + state_publisher_period_))
  {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    last_state_publish_time_ = node_->now();
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

rclcpp_action::GoalResponse JointTrajectoryController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node_->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node_->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::cancel_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Controller uptime
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_DEBUG(
      node_->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::feedback_setup_callback(
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
  rt_goal->preallocated_feedback_->joint_names = joint_names_;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Setup goal status checking timer
  goal_handle_timer_ = node_->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::seconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void JointTrajectoryController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // joint names in the goal are a subset of existing joints, as checked in goal_callback
  // so if the size matches, the goal contains all controller joints
  if (joint_names_.size() == trajectory_msg->joint_names.size())
  {
    return;
  }

  trajectory_msg->joint_names.reserve(joint_names_.size());

  for (size_t index = 0; index < joint_names_.size(); ++index)
  {
    {
      if (
        std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          joint_names_[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(joint_names_[index]);

      for (auto & it : trajectory_msg->points)
      {
        // Assume hold position with 0 velocity and acceleration for missing joints
        it.positions.push_back(joint_command_interface_[0][index].get().get_value());
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
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, joint_names_);
  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & mapping) -> std::vector<double> {
    if (to_remap.empty())
    {
      return to_remap;
    }
    if (to_remap.size() != mapping.size())
    {
      RCLCPP_WARN(node_->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
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
      node_->get_logger(), "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.",
      joint_names_size, string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

bool JointTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_)
  {
    if (trajectory.joint_names.size() != joint_names_.size())
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Empty joint names on incoming trajectory.");
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
    if (trajectory_end_time < node_->now())
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(joint_names_.begin(), joint_names_.end(), incoming_joint_name);
    if (it == joint_names_.end())
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
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
        node_->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    if (
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
  point.positions.resize(size);
  if (has_velocity_state_interface_)
  {
    point.velocities.resize(size);
  }
  if (has_acceleration_state_interface_)
  {
    point.accelerations.resize(size);
  }
}

}  // namespace joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
