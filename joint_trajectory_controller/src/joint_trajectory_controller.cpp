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

#include <cassert>
#include <chrono>
#include <iterator>
#include <string>
#include <memory>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "rcutils/logging_macros.h"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <angles/angles.h>

#include "hardware_interface/utils/time_utils.hpp"

namespace joint_trajectory_controller
{

using namespace std::chrono_literals;
using controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
using lifecycle_msgs::msg::State;

JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(),
  joint_names_({}),
  write_op_names_({})
{}

JointTrajectoryController::JointTrajectoryController(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & write_op_names)
: controller_interface::ControllerInterface(),
  joint_names_(joint_names),
  write_op_names_(write_op_names)
{}

controller_interface::controller_interface_ret_t
JointTrajectoryController::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != CONTROLLER_INTERFACE_RET_SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  lifecycle_node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>("write_op_modes", write_op_names_);
  lifecycle_node_->declare_parameter<double>("state_publish_rate", 50.0);
  lifecycle_node_->declare_parameter<double>("action_monitor_rate", 90.0);
  lifecycle_node_->declare_parameter<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
  declareSegmentTolerances(lifecycle_node_);

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

controller_interface::controller_interface_ret_t
JointTrajectoryController::update()
{
  if (lifecycle_node_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }

  auto resize_joint_trajectory_point = [](trajectory_msgs::msg::JointTrajectoryPoint& point, int size)
  {
    point.positions.resize(size);
    point.velocities.resize(size);
    point.accelerations.resize(size);  
  };

  // current state
  trajectory_msgs::msg::JointTrajectoryPoint state_current;
  size_t joint_num = registered_joint_state_handles_.size();
  resize_joint_trajectory_point(state_current, joint_num);
  for (uint index=0; index<joint_num; ++index)
  {
    auto& joint_state = registered_joint_state_handles_[index];
    state_current.positions[index] = joint_state->get_position();
    state_current.velocities[index] = joint_state->get_velocity();
    state_current.accelerations[index] = 0.0;
  }
  state_current.time_from_start.set__sec(0);

  std::lock_guard<std::mutex> guard(trajectory_mtx_);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->is_empty() == false) {
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      (*traj_point_active_ptr_)->set_point_before_traj(lifecycle_node_->now(), state_current);
    }

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    trajectory_msgs::msg::JointTrajectoryPoint state_desired;
    (*traj_point_active_ptr_)->sample(lifecycle_node_->now(), state_desired,
      start_segment_itr, end_segment_itr);

    trajectory_msgs::msg::JointTrajectoryPoint state_error;
    resize_joint_trajectory_point(state_error, joint_num);

    static bool o = false;
    if (!o)
    {
      RCLCPP_INFO(lifecycle_node_->get_logger(), "NOW sec: %f nsec: %f", lifecycle_node_->now().seconds(), lifecycle_node_->now().nanoseconds());
      auto points = (*traj_point_active_ptr_)->get_trajectory_msg()->points;
      RCLCPP_INFO(lifecycle_node_->get_logger(), "count: %d", points.size());
      auto traj_time = (*traj_point_active_ptr_)->time_from_start();
      RCLCPP_INFO(lifecycle_node_->get_logger(), "TrajStartTime sec: %f nsec %f", traj_time.seconds(), traj_time.nanoseconds());

      for (auto p : points)
      {
        auto timeoffset = static_cast<rclcpp::Duration>(p.time_from_start);
        RCLCPP_INFO(lifecycle_node_->get_logger(), "duration sec: %f nsec %f", timeoffset.seconds(), timeoffset.nanoseconds());
        auto s = hardware_interface::utils::time_add(traj_time, p.time_from_start);
        auto s2 = static_cast<rclcpp::Time>(s);
        RCLCPP_INFO(lifecycle_node_->get_logger(), "POINT sec: %f nsec %f", s2.seconds(), s2.nanoseconds());
        /*RCLCPP_INFO(lifecycle_node_->get_logger(), "%d!", p.positions.size());
        for (auto d : p.positions)
        {
          RCLCPP_INFO(lifecycle_node_->get_logger(), "%f!", d);
        }*/
      }
      o = true;
    }

    if (end_segment_itr != (*traj_point_active_ptr_)->end())
    {
      bool abort = false;
      for (size_t index = 0; index < joint_num; ++index) {
        // set values for next hardware write()
        registered_joint_cmd_handles_[index]->set_cmd(state_desired.positions[index]);
        
        // error defined as the difference between current and desired
        state_error.positions[index] = angles::shortest_angular_distance(
            state_current.positions[index], state_desired.positions[index]);
            
        if (!state_desired.velocities.empty())
          state_error.velocities[index] = state_current.velocities[index] - state_desired.velocities[index];
        else
          state_error.velocities[index] = 0.0;
        state_error.accelerations[index] = 0.0;

        // check tolerances
        bool state_tolerance_ok =
          checkStateTolerancePerJoint(state_error, index, 
            default_tolerances_.state_tolerance[index], false);

        if (!state_tolerance_ok)
        {
          RCLCPP_INFO_ONCE(lifecycle_node_->get_logger(), "abort: %d %.8f", index, state_error.positions[index]);
          abort = true;
        }
      }
      
      // send feedback
      if (rt_active_goal_) {
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = lifecycle_node_->now();
        feedback->joint_names = joint_names_;

        feedback->actual = state_current;
        feedback->desired = state_desired;
        feedback->error = state_error;

        rt_active_goal_->setFeedback(feedback);

        if (abort)
        {
          RCLCPP_INFO(lifecycle_node_->get_logger(), "aborted");
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          rt_active_goal_->setAborted(result);
          rt_active_goal_.reset();
        }
      }
    }
    else
    {
      // past the final point, check that we end up inside goal tolerance
      bool outside_goal_tolerance = false;
      for (size_t index = 0; index < joint_num; ++index) {
        registered_joint_cmd_handles_[index]->set_cmd(state_desired.positions[index]);

        /*RCLCPP_INFO(lifecycle_node_->get_logger(), "index %d: %f %f", index, state_error.positions[index],
            default_tolerances_.goal_state_tolerance[index].position);*/

        if (!checkStateTolerancePerJoint(state_error, index,
          default_tolerances_.goal_state_tolerance[index], true))
        {
          outside_goal_tolerance = true;
          break;
        }
      }

      if (rt_active_goal_)
      {
        if (!outside_goal_tolerance)
        {
          auto res = std::make_shared<FollowJTrajAction::Result>();
          res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
          RCLCPP_INFO(lifecycle_node_->get_logger(), "All joints within range, success!");
          rt_active_goal_->setSucceeded(res);
          rt_active_goal_.reset();
        }
        else
          RCLCPP_INFO(lifecycle_node_->get_logger(), "All joints not within range");
      }
    }

    set_op_mode(hardware_interface::OperationMode::ACTIVE);
  }

  publish_state();
  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  auto logger = lifecycle_node_->get_logger();

  // update parameters
  joint_names_ = lifecycle_node_->get_parameter("joints").as_string_array();
  write_op_names_ = lifecycle_node_->get_parameter("write_op_modes").as_string_array();

  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (auto robot_hardware = robot_hardware_.lock()) {
    if (joint_names_.empty()) {
      RCLCPP_WARN(logger, "no joint names specified");
    }

    // register handles
    registered_joint_state_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_state_handle(
        joint_names_[index].c_str(), &registered_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain joint state handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_joint_cmd_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_command_handle(
        joint_names_[index].c_str(), &registered_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain joint command handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index) {
      auto ret = robot_hardware->get_operation_mode_handle(
        write_op_names_[index].c_str(), &registered_operation_mode_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        RCLCPP_WARN(
          logger, "unable to obtain operation mode handle for %s", write_op_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (
    registered_joint_cmd_handles_.empty() ||
    registered_joint_state_handles_.empty() ||
    registered_operation_mode_handles_.empty())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  default_tolerances_ = getSegmentTolerances(lifecycle_node_, joint_names_);
  RCLCPP_INFO(lifecycle_node_->get_logger(), "tol: %f", default_tolerances_.goal_time_tolerance);

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(registered_joint_state_handles_.size());
  for (size_t index = 0; index < registered_joint_state_handles_.size(); ++index) {
    traj_msg_home_ptr_->points[0].positions[index] =
      registered_joint_state_handles_[index]->get_position();
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_home_point_ptr_ = std::make_shared<Trajectory>();

  // subscriber call back
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this, &logger](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
    -> void
    {
      if (registered_joint_cmd_handles_.size() != msg->joint_names.size()) {
        RCLCPP_ERROR(
          logger,
          "number of joints in joint trajectory msg (%d) "
          "does not match number of joint command handles (%d)\n",
          msg->joint_names.size(), registered_joint_cmd_handles_.size());
      }

      // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
      // always replace old msg with new one for now
      if (subscriber_is_active_) {
        std::lock_guard<std::mutex> guard(trajectory_mtx_);
        traj_external_point_ptr_->update(msg);
      }
    };

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ =
    lifecycle_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // TODO(karsten1987): no lifecyle for subscriber yet
  // joint_command_subscriber_->on_activate();

  // State publisher
  double state_publish_rate =
    lifecycle_node_->get_parameter("state_publish_rate").get_value<double>();
  RCLCPP_INFO_STREAM(
    logger, "Controller state will be published at " <<
      state_publish_rate << "Hz.");
  if (state_publish_rate > 0.0) {
    state_publisher_period_ =
      rclcpp::Duration::from_seconds(1.0 / state_publish_rate);
  } else {
    state_publisher_period_ = rclcpp::Duration(0.0);
  }

  publisher_ = lifecycle_node_->create_publisher<ControllerStateMsg>(
    "state", rclcpp::SystemDefaultsQoS());
  state_publisher_.reset(new StatePublisher(publisher_));

  int n_joints = joint_names_.size();

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

  last_state_publish_time_ = lifecycle_node_->now();

  // action server configuration
  {
    if (lifecycle_node_->has_parameter("allow_partial_joints_goal")) {
      allow_partial_joints_goal_ = lifecycle_node_->get_parameter("allow_partial_joints_goal")
        .get_value<bool>();
    }
    if (allow_partial_joints_goal_) {
      // TODO (ddengster): implement partial joints, log an enabled partial joints goal message
      RCLCPP_WARN(logger, "Warning: Goals with partial set of joints not implemented yet.");
    }

    double action_monitor_rate = lifecycle_node_->get_parameter("action_monitor_rate")
      .get_value<double>();

    RCLCPP_INFO_STREAM(
      logger, "Action status changes will be monitored at " <<
        action_monitor_rate << "Hz.");
    action_monitor_period_ = rclcpp::Duration(1.0 / action_monitor_rate);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      lifecycle_node_->get_node_base_interface(),
      lifecycle_node_->get_node_clock_interface(),
      lifecycle_node_->get_node_logging_interface(),
      lifecycle_node_->get_node_waitables_interface(),
      std::string(lifecycle_node_->get_name()) + "/follow_joint_trajectory",
      std::bind(&JointTrajectoryController::goal_callback, this, _1, _2),
      std::bind(&JointTrajectoryController::cancel_callback, this, _1),
      std::bind(&JointTrajectoryController::feedback_setup_callback, this, _1)
    );
  }

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  is_halted = false;
  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;
  last_state_publish_time_ = lifecycle_node_->now();
  publisher_->on_activate();

  // TODO(karsten1987): activate subscriptions of subscriber
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  subscriber_is_active_ = false;
  publisher_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointTrajectoryController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool
JointTrajectoryController::reset()
{
  // TODO(karsten1987): need a way to re-fetch names after reset. Uncomment this in the future
  // joint_names_.clear();
  // write_op_names_.clear();

  registered_joint_cmd_handles_.clear();
  registered_joint_state_handles_.clear();
  registered_operation_mode_handles_.clear();

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
JointTrajectoryController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  // TODO(karsten1987): what to do?

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
JointTrajectoryController::set_op_mode(const hardware_interface::OperationMode & mode)
{
  for (auto & op_mode_handle : registered_operation_mode_handles_) {
    op_mode_handle->set_mode(mode);
  }
}

void
JointTrajectoryController::halt()
{
  size_t joint_num = registered_joint_cmd_handles_.size();
  for (size_t index = 0; index < joint_num; ++index) {
    registered_joint_cmd_handles_[index]->set_cmd(
      registered_joint_state_handles_[index]->get_position());
  }
  set_op_mode(hardware_interface::OperationMode::ACTIVE);
}

void JointTrajectoryController::publish_state()
{
  if (state_publisher_period_.seconds() <= 0.0) {
    return;
  }

  if (lifecycle_node_->now() < (last_state_publish_time_ + state_publisher_period_)) {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    last_state_publish_time_ = lifecycle_node_->now();

    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    // TODO(ddengster): Fill in the rest of the state data.
    // Port it when we're ready to put in the other trajectory code (ros2_controllers PR #26).

    state_publisher_->unlockAndPublish();
  }
}

rclcpp_action::GoalResponse JointTrajectoryController::goal_callback(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(lifecycle_node_->get_logger(), "Received new action goal");
  (void)uuid;

  // Precondition: Running controller
  if (is_halted) {
    RCLCPP_ERROR(
      lifecycle_node_->get_logger(),
      "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_) {
    if (goal->trajectory.joint_names.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        lifecycle_node_->get_logger(),
        "Joints on incoming goal don't match the controller joints.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(lifecycle_node_->get_logger(), "accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::cancel_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(lifecycle_node_->get_logger(), "Got request to cancel goal");
  (void)goal_handle;

  // Check that cancel request refers to currently active goal (if any)
  if (rt_active_goal_ && rt_active_goal_->gh_ == goal_handle) {
    // Controller uptime
    // TODO(ddengster): Hold position
    // const rclcpp::Time uptime = time_data_.readFromRT()->uptime;
    // Enter hold current position mode
    // setHoldPosition(uptime);

    RCLCPP_DEBUG(
      lifecycle_node_->get_logger(),
      "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    rt_active_goal_->setCanceled(action_res);

    // Reset current goal
    rt_active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::feedback_setup_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(lifecycle_node_->get_logger(), "Doing action handling");

  // Update new trajectory
  {
    std::lock_guard<std::mutex> guard(trajectory_mtx_);
    preempt_active_goal();
    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
      goal_handle->get_goal()->trajectory);
    traj_external_point_ptr_->update(traj_msg);

    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(goal_handle));
    rt_goal->preallocated_feedback_->joint_names = joint_names_;
    rt_active_goal_ = rt_goal;
    rt_active_goal_->execute();
  }

  // Setup goal status checking timer
  goal_handle_timer_ = lifecycle_node_->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::seconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_active_goal_));
}

void JointTrajectoryController::preempt_active_goal()
{
  if (rt_active_goal_) {
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    rt_active_goal_->setCanceled(action_res);
    rt_active_goal_.reset();
  }
}

}  // namespace joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
