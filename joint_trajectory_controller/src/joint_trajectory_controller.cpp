// Copyright 2017 Open Source Robotics Foundation, Inc.
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

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(),
  joint_names_({})
{}

controller_interface::return_type
JointTrajectoryController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);
  node_->declare_parameter<double>("state_publish_rate", 50.0);
  node_->declare_parameter<double>("action_monitor_rate", 20.0);
  node_->declare_parameter<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
  node_->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
  node_->declare_parameter<double>("constraints.goal_time", 0.0);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration JointTrajectoryController::
command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name  : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration JointTrajectoryController::
state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(2 * joint_names_.size());
  for (const auto & joint_name  : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::return_type
JointTrajectoryController::update()
{
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  auto resize_joint_trajectory_point =
    [](trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
    {
      point.positions.resize(size);
      point.velocities.resize(size);
      point.accelerations.resize(size);
    };
  auto compute_error_for_joint = [](JointTrajectoryPoint & error, int index,
      const JointTrajectoryPoint & current, const JointTrajectoryPoint & desired)
    {
      // error defined as the difference between current and desired
      error.positions[index] = angles::shortest_angular_distance(
        current.positions[index], desired.positions[index]);
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
      error.accelerations[index] = 0.0;
    };

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    traj_external_point_ptr_->update(*new_external_msg);
  }

  JointTrajectoryPoint state_current, state_desired, state_error;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(state_current, joint_num);

  // current state update
  for (auto index = 0ul; index < joint_num; ++index) {
    state_current.positions[index] = joint_position_state_interface_[index].get().get_value();
    state_current.velocities[index] = joint_velocity_state_interface_[index].get().get_value();
    state_current.accelerations[index] = 0.0;
  }
  state_current.time_from_start.set__sec(0);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg() == false) {
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already()) {
      (*traj_point_active_ptr_)->set_point_before_trajectory_msg(
        node_->now(), state_current);
    }
    resize_joint_trajectory_point(state_error, joint_num);

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point = (*traj_point_active_ptr_)->sample(
      node_->now(), state_desired,
      start_segment_itr, end_segment_itr);

    if (valid_point) {
      bool abort = false;
      bool outside_goal_tolerance = false;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();
      for (auto index = 0ul; index < joint_num; ++index) {
        // set values for next hardware write()
        joint_position_command_interface_[index].get().set_value(state_desired.positions[index]);
        compute_error_for_joint(state_error, index, state_current, state_desired);

        if (before_last_point && !check_state_tolerance_per_joint(
            state_error, index,
            default_tolerances_.state_tolerance[index], false))
        {
          abort = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point && !check_state_tolerance_per_joint(
            state_error, index,
            default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;
        }
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = node_->now();
        feedback->joint_names = joint_names_;

        feedback->actual = state_current;
        feedback->desired = state_desired;
        feedback->error = state_error;
        active_goal->setFeedback(feedback);

        // check abort
        if (abort || outside_goal_tolerance) {
          auto result = std::make_shared<FollowJTrajAction::Result>();

          if (abort) {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to state tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          } else if (outside_goal_tolerance) {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to goal tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
          }
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

          // check goal tolerance
        } else if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

            RCLCPP_INFO(node_->get_logger(), "Goal reached, success!");
          } else if (default_tolerances_.goal_time_tolerance != 0.0) {
            // if we exceed goal_time_toleralance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            const double difference = node_->now().seconds() - traj_end.seconds();
            if (difference > default_tolerances_.goal_time_tolerance) {
              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
              active_goal->setAborted(result);
              // TODO(matthew-reynolds): Need a lock-free write here
              // See https://github.com/ros-controls/ros2_controllers/issues/168
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              RCLCPP_WARN(
                node_->get_logger(),
                "Aborted due goal_time_tolerance exceeding by %f seconds",
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();

  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
    RCLCPP_WARN(logger, "no joint names specified");
  }

  default_tolerances_ = get_segment_tolerances(*node_, joint_names_);

  // subscriber call back
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
    -> void
    {
      if (!validate_trajectory_msg(*msg)) {
        return;
      }

      // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
      // always replace old msg with new one for now
      if (subscriber_is_active_) {
        add_new_trajectory_msg(msg);
      }
    };

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ =
    node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // TODO(karsten1987): no lifecyle for subscriber yet
  // joint_command_subscriber_->on_activate();

  // State publisher
  const double state_publish_rate =
    node_->get_parameter("state_publish_rate").get_value<double>();
  RCLCPP_INFO_STREAM(
    logger, "Controller state will be published at " <<
      state_publish_rate << "Hz.");
  if (state_publish_rate > 0.0) {
    state_publisher_period_ =
      rclcpp::Duration::from_seconds(1.0 / state_publish_rate);
  } else {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ = node_->create_publisher<ControllerStateMsg>(
    "state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  const auto n_joints = joint_names_.size();

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.desired.positions.resize(n_joints);
  state_publisher_->msg_.desired.velocities.resize(n_joints);
  state_publisher_->msg_.desired.accelerations.resize(n_joints);
  state_publisher_->msg_.actual.positions.resize(n_joints);
  state_publisher_->msg_.actual.velocities.resize(n_joints);
  state_publisher_->msg_.error.positions.resize(n_joints);
  state_publisher_->msg_.error.velocities.resize(n_joints);
  state_publisher_->unlock();

  last_state_publish_time_ = node_->now();

  // action server configuration
  allow_partial_joints_goal_ = node_->get_parameter("allow_partial_joints_goal")
    .get_value<bool>();
  if (allow_partial_joints_goal_) {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  const double action_monitor_rate = node_->get_parameter("action_monitor_rate")
    .get_value<double>();

  RCLCPP_INFO_STREAM(
    logger, "Action status changes will be monitored at " <<
      action_monitor_rate << "Hz.");
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    std::string(node_->get_name()) + "/follow_joint_trajectory",
    std::bind(&JointTrajectoryController::goal_callback, this, _1, _2),
    std::bind(&JointTrajectoryController::cancel_callback, this, _1),
    std::bind(&JointTrajectoryController::feedback_setup_callback, this, _1)
  );

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if ((command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_command_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position command interfaces, got %u",
      joint_names_.size(), joint_position_command_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position state interfaces, got %u",
      joint_names_.size(), joint_position_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY,
      joint_velocity_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u velocity state interfaces, got %u",
      joint_names_.size(), joint_velocity_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(joint_position_state_interface_.size());
  for (size_t index = 0; index < joint_position_state_interface_.size(); ++index) {
    traj_msg_home_ptr_->points[0].positions[index] =
      joint_position_state_interface_[index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_home_point_ptr_ = std::make_shared<Trajectory>();

  is_halted = false;
  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;
  last_state_publish_time_ = node_->now();

  // TODO(karsten1987): activate subscriptions of subscriber
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  joint_position_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  release_interfaces();

  subscriber_is_active_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool
JointTrajectoryController::reset()
{
  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();

  // iterator has no default value
  // prev_traj_point_ptr_;
  traj_point_active_ptr_ = nullptr;
  traj_external_point_ptr_.reset();
  traj_home_point_ptr_.reset();
  traj_msg_home_ptr_.reset();

  is_halted = false;

  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_shutdown(const rclcpp_lifecycle::State &)
{
  // TODO(karsten1987): what to do?

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
JointTrajectoryController::halt()
{
  const size_t joint_num = joint_position_command_interface_.size();
  for (size_t index = 0; index < joint_num; ++index) {
    joint_position_command_interface_[index].get().set_value(
      joint_position_command_interface_[index].get().get_value());
  }
}

void JointTrajectoryController::publish_state(
  const JointTrajectoryPoint & desired_state,
  const JointTrajectoryPoint & current_state, const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0) {
    return;
  }

  if (node_->now() < (last_state_publish_time_ + state_publisher_period_)) {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    last_state_publish_time_ = node_->now();
    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    state_publisher_->msg_.desired.positions = desired_state.positions;
    state_publisher_->msg_.desired.velocities = desired_state.velocities;
    state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
    state_publisher_->msg_.actual.positions = current_state.positions;
    state_publisher_->msg_.actual.velocities = current_state.velocities;
    state_publisher_->msg_.error.positions = state_error.positions;
    state_publisher_->msg_.error.velocities = state_error.velocities;

    state_publisher_->unlockAndPublish();
  }
}

rclcpp_action::GoalResponse JointTrajectoryController::goal_callback(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (is_halted) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory)) {
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
  if (active_goal && active_goal->gh_ == goal_handle) {
    // Controller uptime
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_DEBUG(
      node_->get_logger(),
      "Canceling active action goal because cancel callback received.");

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
    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
      goal_handle->get_goal()->trajectory);

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
  if (joint_names_.size() == trajectory_msg->joint_names.size()) {
    return;
  }

  trajectory_msg->joint_names.reserve(joint_names_.size());

  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    {
      if (std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          joint_names_[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(joint_names_[index]);

      const auto & joint_state = joint_position_command_interface_[index];
      for (auto & it : trajectory_msg->points) {
        // Assume hold position with 0 velocity and acceleration for missing joints
        it.positions.push_back(joint_state.get().get_value());
        if (!it.velocities.empty()) {
          it.velocities.push_back(0.0);
        }
        if (!it.accelerations.empty()) {
          it.accelerations.push_back(0.0);
        }
        if (!it.effort.empty()) {
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
  auto remap = [this](const std::vector<double> & to_remap, const std::vector<size_t> & mapping)
    -> std::vector<double>
    {
      if (to_remap.empty()) {
        return to_remap;
      }
      if (to_remap.size() != mapping.size()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Invalid input size (%d) for sorting", to_remap.size());
        return to_remap;
      }
      std::vector<double> output;
      output.resize(mapping.size(), 0.0);
      for (auto index = 0ul; index < mapping.size(); ++index) {
        auto map_index = mapping[index];
        output[map_index] = to_remap[index];
      }
      return output;
    };

  for (auto index = 0ul; index < trajectory_msg->points.size(); ++index) {
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
  size_t joint_names_size,
  const std::vector<double> & vector_field,
  const std::string & string_for_vector_field, size_t i, bool allow_empty) const
{
  if (allow_empty && vector_field.empty()) {
    return true;
  }
  if (joint_names_size != vector_field.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Mismatch between joint_names (%u) and %s (%u) at point #%u.",
      joint_names_size, string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}


bool JointTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_) {
    if (trajectory.joint_names.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  if (trajectory.joint_names.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Empty joint names on incoming trajectory.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0) {
    auto trajectory_end_time = trajectory_start_time;
    for (const auto & p : trajectory.points) {
      trajectory_end_time += p.time_from_start;
    }
    if (trajectory_end_time < node_->now()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (auto i = 0ul; i < trajectory.joint_names.size(); ++i) {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(joint_names_.begin(), joint_names_.end(), incoming_joint_name);
    if (it == joint_names_.end()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  rclcpp::Duration previous_traj_time(0);
  for (auto i = 0ul; i < trajectory.points.size(); ++i) {
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Time between points %u and %u is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    if (!validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].accelerations, "accelerations", i,
        true) ||
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
  if (active_goal) {
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

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
    empty_msg);
  add_new_trajectory_msg(traj_msg);
}

}  // namespace joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
