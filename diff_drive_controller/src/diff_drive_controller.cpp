// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include "diff_drive_controller/diff_drive_controller.hpp"

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <limits>

#include "diff_drive_controller/urdf_util.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_WHEEL_JOINT_CONTROLLER_STATE_TOPIC = "wheel_joint_controller_state";
}  // namespace

namespace diff_drive_controller
{
using namespace std::chrono_literals;
using CallbackReturn = DiffDriveController::CallbackReturn;
using lifecycle_msgs::msg::State;

DiffDriveController::DiffDriveController()
: controller_interface::ControllerInterface() {}

DiffDriveController::DiffDriveController(
  std::vector<std::string> left_wheel_names, std::vector<std::string> right_wheel_names,
  std::vector<std::string> write_op_names)
: controller_interface::ControllerInterface(),
  left_wheel_names_(std::move(left_wheel_names)),
  right_wheel_names_(std::move(right_wheel_names)),
  write_op_names_(std::move(write_op_names))
{
}

controller_interface::return_type
DiffDriveController::init(
  std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
  const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "left_wheel_names", left_wheel_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "right_wheel_names", right_wheel_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>("write_op_modes", write_op_names_);

  lifecycle_node_->declare_parameter<double>("wheel_separation", 0.0);
  lifecycle_node_->declare_parameter<double>("wheel_radius", 0.0);
  lifecycle_node_->declare_parameter<double>("wheel_separation_multiplier", 1.0);
  lifecycle_node_->declare_parameter<double>("left_wheel_radius_multiplier", 1.0);
  lifecycle_node_->declare_parameter<double>("right_wheel_radius_multiplier", 1.0);
  lifecycle_node_->declare_parameter("robot_description");

  lifecycle_node_->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
  lifecycle_node_->declare_parameter<std::string>("base_frame_id", base_frame_id_);
  lifecycle_node_->declare_parameter<std::vector<double>>("pose_covariance_diagonal", {});
  lifecycle_node_->declare_parameter<std::vector<double>>("twist_covariance_diagonal", {});
  lifecycle_node_->declare_parameter<bool>("open_loop", open_loop_);
  lifecycle_node_->declare_parameter<bool>("enable_odom_tf", enable_odom_tf_);

  lifecycle_node_->declare_parameter<int>("cmd_vel_timeout", cmd_vel_timeout_.count());
  lifecycle_node_->declare_parameter<bool>(
    "allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_);
  lifecycle_node_->declare_parameter<bool>("publish_limited_velocity", publish_limited_velocity_);
  lifecycle_node_->declare_parameter<bool>(
    "publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_);
  lifecycle_node_->declare_parameter<int>("velocity_rolling_window_size", 10);

  lifecycle_node_->declare_parameter<bool>("linear.x.has_velocity_limits", false);
  lifecycle_node_->declare_parameter<bool>("linear.x.has_acceleration_limits", false);
  lifecycle_node_->declare_parameter<bool>("linear.x.has_jerk_limits", false);
  lifecycle_node_->declare_parameter<double>("linear.x.max_velocity", 0.0);
  lifecycle_node_->declare_parameter<double>("linear.x.min_velocity", 0.0);
  lifecycle_node_->declare_parameter<double>("linear.x.max_acceleration", 0.0);
  lifecycle_node_->declare_parameter<double>("linear.x.min_acceleration", 0.0);
  lifecycle_node_->declare_parameter<double>("linear.x.max_jerk", 0.0);
  lifecycle_node_->declare_parameter<double>("linear.x.min_jerk", 0.0);

  lifecycle_node_->declare_parameter<bool>("angular.z.has_velocity_limits", false);
  lifecycle_node_->declare_parameter<bool>("angular.z.has_acceleration_limits", false);
  lifecycle_node_->declare_parameter<bool>("angular.z.has_jerk_limits", false);
  lifecycle_node_->declare_parameter<double>("angular.z.max_velocity", 0.0);
  lifecycle_node_->declare_parameter<double>("angular.z.min_velocity", 0.0);
  lifecycle_node_->declare_parameter<double>("angular.z.max_acceleration", 0.0);
  lifecycle_node_->declare_parameter<double>("angular.z.min_acceleration", 0.0);
  lifecycle_node_->declare_parameter<double>("angular.z.max_jerk", 0.0);
  lifecycle_node_->declare_parameter<double>("angular.z.min_jerk", 0.0);

  return controller_interface::return_type::SUCCESS;
}

controller_interface::return_type DiffDriveController::update()
{
  auto logger = lifecycle_node_->get_logger();
  if (lifecycle_node_->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::SUCCESS;
  }

  const auto current_time = lifecycle_node_->get_clock()->now();

  if (open_loop_) {
    odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, current_time);
  } else {
    double left_position_mean = 0.0;
    double right_position_mean = 0.0;
    for (size_t index = 0; index < wheels_per_side_; ++index) {
      const double left_position = registered_left_wheel_handles_[index].state->get_position();
      const double right_position = registered_right_wheel_handles_[index].state->get_position();

      if (std::isnan(left_position) || std::isnan(right_position)) {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel position is invalid for index [%d]",
          index);
        return controller_interface::return_type::ERROR;
      }

      left_position_mean += left_position;
      right_position_mean += right_position;
    }
    left_position_mean /= wheels_per_side_;
    right_position_mean /= wheels_per_side_;

    odometry_.update(left_position_mean, right_position_mean, current_time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  publish_odom(current_time, orientation);
  publish_odom_tf(current_time, orientation);

  // Retrieve last velocity command received
  Commands curr_cmd = *(command_.readFromRT());

  // Brake if cmd_vel has timeout
  if (current_time - curr_cmd.stamp > cmd_vel_timeout_) {
    curr_cmd.lin = 0.0;
    curr_cmd.ang = 0.0;
  }

  // Time elapsed since last update iteration
  const auto update_dt = current_time - previous_update_timestamp_;

  // NOTE: publish_wheel_joint_controller_state() must be called before limiter is applied.
  publish_wheel_joint_controller_state(current_time, update_dt, curr_cmd);

  // Enforce limiters
  limiter_linear_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, update_dt.seconds());
  limiter_angular_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, update_dt.seconds());

  publish_cmd_vel_out(current_time, curr_cmd);

  // Compute wheels velocities:
  const double velocity_left =
    (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0) / left_wheel_radius_;
  const double velocity_right =
    (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0) / right_wheel_radius_;

  // Send wheel velocity commands:
  for (size_t index = 0; index < wheels_per_side_; ++index) {
    registered_left_wheel_handles_[index].command->set_cmd(velocity_left);
    registered_right_wheel_handles_[index].command->set_cmd(velocity_right);
  }

  set_op_mode(hardware_interface::OperationMode::ACTIVE);

  // push back command history
  last1_cmd_ = last0_cmd_;
  last0_cmd_ = curr_cmd;
  previous_update_timestamp_ = current_time;

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn DiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  auto logger = lifecycle_node_->get_logger();

  left_wheel_names_ = lifecycle_node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = lifecycle_node_->get_parameter("right_wheel_names").as_string_array();

  if (left_wheel_names_.size() != right_wheel_names_.size()) {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%d] and the number of right wheels [%d] are different",
      left_wheel_names_.size(), right_wheel_names_.size());
    return CallbackReturn::ERROR;
  }

  auto raw_separation = lifecycle_node_->get_parameter("wheel_separation").as_double();
  auto raw_radius = lifecycle_node_->get_parameter("wheel_radius").as_double();
  auto separation_multiplier =
    lifecycle_node_->get_parameter("wheel_separation_multiplier").as_double();
  auto left_radius_multiplier =
    lifecycle_node_->get_parameter("left_wheel_radius_multiplier").as_double();
  auto right_radius_multiplier =
    lifecycle_node_->get_parameter("right_wheel_radius_multiplier").as_double();

  if (!set_wheel_params_from_urdf(
      left_wheel_names_[0], right_wheel_names_[0], raw_separation, raw_radius))
  {
    RCLCPP_ERROR_STREAM(
      logger, "The following configurations must be set via parameter or urdf: " <<
      (raw_separation == 0.0 ? "'wheel_separation' " : "") <<
      (raw_radius == 0.0 ? "'wheel_radius'" : ""));
    return CallbackReturn::FAILURE;
  }

  wheel_separation_ = separation_multiplier * raw_separation;
  left_wheel_radius_ = left_radius_multiplier * raw_radius;
  right_wheel_radius_ = right_radius_multiplier * raw_radius;
  wheels_per_side_ = left_wheel_names_.size();

  odometry_.setWheelParams(wheel_separation_, left_wheel_radius_, right_wheel_radius_);
  odometry_.setVelocityRollingWindowSize(
    lifecycle_node_->get_parameter("velocity_rolling_window_size").as_int());

  odom_frame_id_ = lifecycle_node_->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = lifecycle_node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = lifecycle_node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(pose_diagonal.begin(), pose_diagonal.end(), pose_covariance_diagonal_.begin());

  auto twist_diagonal =
    lifecycle_node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(twist_diagonal.begin(), twist_diagonal.end(), twist_covariance_diagonal_.begin());

  open_loop_ = lifecycle_node_->get_parameter("open_loop").as_bool();
  enable_odom_tf_ = lifecycle_node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ =
    std::chrono::milliseconds{lifecycle_node_->get_parameter("cmd_vel_timeout").as_int()};
  allow_multiple_cmd_vel_publishers_ =
    lifecycle_node_->get_parameter("allow_multiple_cmd_vel_publishers").as_bool();
  RCLCPP_INFO_STREAM(
    logger, "Allow multiple cmd_vel publishers is " <<
    (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));

  publish_limited_velocity_ = lifecycle_node_->get_parameter("publish_limited_velocity").as_bool();
  publish_wheel_joint_controller_state_ =
    lifecycle_node_->get_parameter("publish_wheel_joint_controller_state").as_bool();

  limiter_linear_ = SpeedLimiter(
    lifecycle_node_->get_parameter("linear.x.has_velocity_limits").as_bool(),
    lifecycle_node_->get_parameter("linear.x.has_acceleration_limits").as_bool(),
    lifecycle_node_->get_parameter("linear.x.has_jerk_limits").as_bool(),
    lifecycle_node_->get_parameter("linear.x.min_velocity").as_double(),
    lifecycle_node_->get_parameter("linear.x.max_velocity").as_double(),
    lifecycle_node_->get_parameter("linear.x.min_acceleration").as_double(),
    lifecycle_node_->get_parameter("linear.x.max_acceleration").as_double(),
    lifecycle_node_->get_parameter("linear.x.min_jerk").as_double(),
    lifecycle_node_->get_parameter("linear.x.max_jerk").as_double());

  limiter_angular_ = SpeedLimiter(
    lifecycle_node_->get_parameter("angular.z.has_velocity_limits").as_bool(),
    lifecycle_node_->get_parameter("angular.z.has_acceleration_limits").as_bool(),
    lifecycle_node_->get_parameter("angular.z.has_jerk_limits").as_bool(),
    lifecycle_node_->get_parameter("angular.z.min_velocity").as_double(),
    lifecycle_node_->get_parameter("angular.z.max_velocity").as_double(),
    lifecycle_node_->get_parameter("angular.z.min_acceleration").as_double(),
    lifecycle_node_->get_parameter("angular.z.max_acceleration").as_double(),
    lifecycle_node_->get_parameter("angular.z.min_jerk").as_double(),
    lifecycle_node_->get_parameter("angular.z.max_jerk").as_double());

  if (auto robot_hardware = robot_hardware_.lock()) {
    const auto left_result =
      configure_side("left", left_wheel_names_, registered_left_wheel_handles_, *robot_hardware);
    const auto right_result =
      configure_side("right", right_wheel_names_, registered_right_wheel_handles_, *robot_hardware);

    if (left_result == CallbackReturn::FAILURE || right_result == CallbackReturn::FAILURE) {
      return CallbackReturn::FAILURE;
    }

    write_op_names_ = lifecycle_node_->get_parameter("write_op_modes").as_string_array();

    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index) {
      const auto op_name = write_op_names_[index].c_str();
      auto & op_handle = registered_operation_mode_handles_[index];

      auto result = robot_hardware->get_operation_mode_handle(op_name, &op_handle);
      if (result != hardware_interface::return_type::OK) {
        RCLCPP_WARN(logger, "unable to obtain operation mode handle for %s", op_name);
        return CallbackReturn::FAILURE;
      }
    }
  } else {
    return CallbackReturn::ERROR;
  }

  if (
    registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty() ||
    registered_operation_mode_handles_.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Either left wheel handles, right wheel handles, or operation modes are non existant");
    return CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_) {
    limited_velocity_publisher_ = lifecycle_node_->create_publisher<Twist>(
      DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  if (publish_wheel_joint_controller_state_) {
    wheel_joint_controller_state_publisher_ =
      lifecycle_node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      DEFAULT_WHEEL_JOINT_CONTROLLER_STATE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_wheel_joint_controller_state_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(
      wheel_joint_controller_state_publisher_);

    const size_t num_wheels = wheels_per_side_ * 2;

    realtime_wheel_joint_controller_state_publisher_->msg_.joint_names.resize(num_wheels);

    realtime_wheel_joint_controller_state_publisher_->msg_.desired.positions.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.desired.velocities.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.desired.accelerations.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.desired.effort.resize(num_wheels);

    realtime_wheel_joint_controller_state_publisher_->msg_.actual.positions.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.actual.velocities.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.actual.accelerations.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.actual.effort.resize(num_wheels);

    realtime_wheel_joint_controller_state_publisher_->msg_.error.positions.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.error.velocities.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.error.accelerations.resize(num_wheels);
    realtime_wheel_joint_controller_state_publisher_->msg_.error.effort.resize(num_wheels);

    for (size_t i = 0; i < wheels_per_side_; ++i) {
      realtime_wheel_joint_controller_state_publisher_->msg_.joint_names[i] = left_wheel_names_[i];
      realtime_wheel_joint_controller_state_publisher_->msg_.joint_names[i + wheels_per_side_] =
        right_wheel_names_[i];
    }

    vel_left_actual_previous_.resize(wheels_per_side_, 0.0);
    vel_right_actual_previous_.resize(wheels_per_side_, 0.0);
  }

  // Zero-initialize command
  command_.initRT(Commands());

  // Fill last two commands with default constructed commands
  last0_cmd_ = Commands();
  last1_cmd_ = Commands();

  // initialize command subscriber
  velocity_command_subscriber_ = lifecycle_node_->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Twist> msg) -> void {on_cmd_vel(msg);});

  // initialize odometry publisher and message
  odometry_publisher_ = lifecycle_node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id_;
  odometry_message.child_frame_id = base_frame_id_;

  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance();

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = pose_covariance_diagonal_[index];
    odometry_message.twist.covariance[diagonal_index] = twist_covariance_diagonal_[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = lifecycle_node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id_;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id_;

  previous_update_timestamp_ = lifecycle_node_->get_clock()->now();
  set_op_mode(hardware_interface::OperationMode::INACTIVE);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  is_halted = false;
  subscriber_is_active_ = true;

  odometry_transform_publisher_->on_activate();
  odometry_publisher_->on_activate();
  if (publish_limited_velocity_) {
    limited_velocity_publisher_->on_activate();
  }
  if (publish_wheel_joint_controller_state_) {
    wheel_joint_controller_state_publisher_->on_activate();
  }

  RCLCPP_INFO(
    lifecycle_node_->get_logger(), "Lifecycle subscriber and publisher are currently active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  halt();
  subscriber_is_active_ = false;
  odometry_transform_publisher_->on_deactivate();
  odometry_publisher_->on_deactivate();
  if (publish_limited_velocity_) {
    limited_velocity_publisher_->on_deactivate();
  }
  if (publish_wheel_joint_controller_state_) {
    wheel_joint_controller_state_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  halt();
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool DiffDriveController::reset()
{
  odometry_.resetOdometry();

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_operation_mode_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  odometry_publisher_.reset();
  realtime_odometry_publisher_.reset();
  odometry_transform_publisher_.reset();
  realtime_odometry_transform_publisher_.reset();
  limited_velocity_publisher_.reset();
  realtime_limited_velocity_publisher_.reset();
  wheel_joint_controller_state_publisher_.reset();
  realtime_wheel_joint_controller_state_publisher_.reset();

  is_halted = false;
  return true;
}

CallbackReturn DiffDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  halt();
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void DiffDriveController::set_op_mode(const hardware_interface::OperationMode & mode)
{
  for (auto & op_mode_handle : registered_operation_mode_handles_) {
    op_mode_handle->set_mode(mode);
  }
}

void DiffDriveController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles) {
      for (const auto & wheel_handle : wheel_handles) {
        wheel_handle.command->set_cmd(0.0);
      }
    };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
  set_op_mode(hardware_interface::OperationMode::ACTIVE);
}

CallbackReturn DiffDriveController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles, hardware_interface::RobotHardware & robot_hardware)
{
  auto logger = lifecycle_node_->get_logger();

  if (wheel_names.empty()) {
    std::stringstream ss;
    ss << "No " << side << " wheel names specified.";
    RCLCPP_ERROR(logger, ss.str().c_str());
    return CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.resize(wheel_names.size());
  for (size_t index = 0; index < wheel_names.size(); ++index) {
    const auto wheel_name = wheel_names[index].c_str();
    auto & wheel_handle = registered_handles[index];

    auto result = robot_hardware.get_joint_state_handle(wheel_name, &wheel_handle.state);
    if (result != hardware_interface::return_type::OK) {
      RCLCPP_WARN(logger, "unable to obtain joint state handle for %s", wheel_name);
      return CallbackReturn::FAILURE;
    }

    auto ret = robot_hardware.get_joint_command_handle(wheel_name, &wheel_handle.command);
    if (ret != hardware_interface::return_type::OK) {
      RCLCPP_WARN(logger, "unable to obtain joint command handle for %s", wheel_name);
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

bool DiffDriveController::set_wheel_params_from_urdf(
  const std::string & left_wheel_name, const std::string & right_wheel_name, double & separation,
  double & radius)
{
  auto lookup_wheel_separation = separation == 0.0;
  auto lookup_wheel_radius = radius == 0.0;
  if (!(lookup_wheel_separation || lookup_wheel_radius)) {
    // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
    return true;
  }

  // Parse robot description
  const std::string model_param_name = "robot_description";
  std::string robot_model_str;
  if (!lifecycle_node_->get_parameter(model_param_name, robot_model_str)) {
    RCLCPP_ERROR(
      lifecycle_node_->get_logger(), "Robot description couldn't be retrieved from param server.");
    return false;
  }

  urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

  urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
  urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));

  if (lookup_wheel_separation) {
    // Get wheel separation
    if (!left_wheel_joint) {
      RCLCPP_ERROR_STREAM(
        lifecycle_node_->get_logger(),
        left_wheel_name << " couldn't be retrieved from model description");
      return false;
    }

    if (!right_wheel_joint) {
      RCLCPP_ERROR_STREAM(
        lifecycle_node_->get_logger(),
        right_wheel_name << " couldn't be retrieved from model description");
      return false;
    }

    RCLCPP_INFO_STREAM(
      lifecycle_node_->get_logger(),
      "left wheel to origin: " << left_wheel_joint->parent_to_joint_origin_transform.position.x <<
        "," <<
        left_wheel_joint->parent_to_joint_origin_transform.position.y <<
        ", " <<
        left_wheel_joint->parent_to_joint_origin_transform.position.z);
    RCLCPP_INFO_STREAM(
      lifecycle_node_->get_logger(),
      "right wheel to origin: " << right_wheel_joint->parent_to_joint_origin_transform.position.x <<
        "," <<
        right_wheel_joint->parent_to_joint_origin_transform.position.y <<
        ", " <<
        right_wheel_joint->parent_to_joint_origin_transform.position.z);

    separation = urdf_util::euclidean_of_vectors(
      left_wheel_joint->parent_to_joint_origin_transform.position,
      right_wheel_joint->parent_to_joint_origin_transform.position);
  }

  if (lookup_wheel_radius) {
    // Get wheel radius
    if (!urdf_util::get_wheel_radius(
        model->getLink(left_wheel_joint->child_link_name), radius,
        lifecycle_node_->get_logger()))
    {
      RCLCPP_ERROR_STREAM(
        lifecycle_node_->get_logger(), "Couldn't retrieve " << left_wheel_name << " wheel radius");
      return false;
    }
  }

  return true;
}

void DiffDriveController::on_cmd_vel(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  if (!subscriber_is_active_) {
    RCLCPP_WARN(lifecycle_node_->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  auto & clk = *lifecycle_node_->get_clock().get();
  if (
    !allow_multiple_cmd_vel_publishers_ &&
    velocity_command_subscriber_->get_publisher_count() > 1)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(
      lifecycle_node_->get_logger(), clk, 1000,
      "Detected " << velocity_command_subscriber_->get_publisher_count() <<
        " publishers. Only 1 publisher is allowed. Going to brake.");
    halt();
    return;
  }

  if (!std::isfinite(msg->twist.angular.z) || !std::isfinite(msg->twist.linear.x)) {
    RCLCPP_WARN_THROTTLE(
      lifecycle_node_->get_logger(), clk, 1000, "Received NaN in velocity command. Ignoring.");
    return;
  }

  command_struct_.ang = msg->twist.angular.z;
  command_struct_.lin = msg->twist.linear.x;
  command_struct_.stamp = clk.now();
  command_.writeFromNonRT(command_struct_);
  RCLCPP_DEBUG_STREAM(
    lifecycle_node_->get_logger(), "Added values to command. " <<
      "Ang: " << command_struct_.ang << ", " <<
      "Lin: " << command_struct_.lin << ", " <<
      "Stamp: " << command_struct_.stamp.seconds());
}

void DiffDriveController::publish_odom(
  const rclcpp::Time & current_time, const tf2::Quaternion & orientation) const
{
  if (odometry_publisher_->is_activated() && realtime_odometry_publisher_->trylock()) {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = current_time;
    odometry_message.pose.pose.position.x = odometry_.getX();
    odometry_message.pose.pose.position.y = odometry_.getY();
    odometry_message.pose.pose.orientation.x = orientation.x();
    odometry_message.pose.pose.orientation.y = orientation.y();
    odometry_message.pose.pose.orientation.z = orientation.z();
    odometry_message.pose.pose.orientation.w = orientation.w();
    odometry_message.twist.twist.linear.x = odometry_.getLinear();
    odometry_message.twist.twist.angular.z = odometry_.getAngular();
    realtime_odometry_publisher_->unlockAndPublish();
  }
}

void DiffDriveController::publish_odom_tf(
  const rclcpp::Time & current_time, const tf2::Quaternion & orientation) const
{
  if (
    enable_odom_tf_ && odometry_transform_publisher_->is_activated() &&
    realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = current_time;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }
}

void DiffDriveController::publish_cmd_vel_out(
  const rclcpp::Time & current_time, const DiffDriveController::Commands & curr_cmd) const
{
  if (
    publish_limited_velocity_ && limited_velocity_publisher_->is_activated() &&
    realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist.linear.x = curr_cmd.lin;
    limited_velocity_command.twist.angular.z = curr_cmd.ang;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }
}

void DiffDriveController::publish_wheel_joint_controller_state(
  const rclcpp::Time & time, const rclcpp::Duration & period, const Commands & curr_cmd)
{
  if (
    publish_wheel_joint_controller_state_ &&
    wheel_joint_controller_state_publisher_->is_activated() &&
    realtime_wheel_joint_controller_state_publisher_->trylock())
  {
    const auto cmd_dt = period.seconds();

    // Compute desired wheels velocities, that is before applying limits:
    const auto vel_left_desired =
      (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0) / left_wheel_radius_;
    const auto vel_right_desired =
      (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0) / right_wheel_radius_;

    auto & msg = realtime_wheel_joint_controller_state_publisher_->msg_;
    msg.header.stamp = time;

    for (size_t i = 0; i < wheels_per_side_; ++i) {
      const auto control_duration = (time - previous_update_timestamp_).seconds();

      auto vel_left_actual = registered_left_wheel_handles_[i].state->get_velocity();
      const auto left_wheel_acc =
        (vel_left_actual - vel_left_actual_previous_[i]) / control_duration;
      const auto right_wheel_acc =
        (registered_right_wheel_handles_[i].state->get_velocity() - vel_right_actual_previous_[i]) /
        control_duration;

      // Actual
      msg.actual.positions[i] = registered_left_wheel_handles_[i].state->get_position();
      msg.actual.velocities[i] = vel_left_actual;
      msg.actual.accelerations[i] = left_wheel_acc;
      msg.actual.effort[i] = registered_left_wheel_handles_[i].state->get_effort();

      msg.actual.positions[i + wheels_per_side_] =
        registered_right_wheel_handles_[i].state->get_position();
      msg.actual.velocities[i + wheels_per_side_] =
        registered_right_wheel_handles_[i].state->get_velocity();
      msg.actual.accelerations[i + wheels_per_side_] = right_wheel_acc;
      msg.actual.effort[i + wheels_per_side_] =
        registered_right_wheel_handles_[i].state->get_effort();

      // Desired
      msg.desired.positions[i] += vel_left_desired * cmd_dt;
      msg.desired.velocities[i] = vel_left_desired;
      msg.desired.accelerations[i] = (vel_left_desired - vel_left_desired_previous_) * cmd_dt;
      msg.desired.effort[i] = std::numeric_limits<double>::quiet_NaN();

      msg.desired.positions[i + wheels_per_side_] += vel_right_desired * cmd_dt;
      msg.desired.velocities[i + wheels_per_side_] = vel_right_desired;
      msg.desired.accelerations[i + wheels_per_side_] =
        (vel_right_desired - vel_right_desired_previous_) * cmd_dt;
      msg.desired.effort[i + wheels_per_side_] = std::numeric_limits<double>::quiet_NaN();

      // Error
      msg.error.positions[i] = msg.desired.positions[i] - msg.actual.positions[i];
      msg.error.velocities[i] = msg.desired.velocities[i] - msg.actual.velocities[i];
      msg.error.accelerations[i] = msg.desired.accelerations[i] - msg.actual.accelerations[i];
      msg.error.effort[i] = msg.desired.effort[i] - msg.actual.effort[i];

      msg.error.positions[i + wheels_per_side_] =
        msg.desired.positions[i + wheels_per_side_] - msg.actual.positions[i + wheels_per_side_];
      msg.error.velocities[i + wheels_per_side_] =
        msg.desired.velocities[i + wheels_per_side_] - msg.actual.velocities[i + wheels_per_side_];
      msg.error.accelerations[i + wheels_per_side_] =
        msg.desired.accelerations[i + wheels_per_side_] -
        msg.actual.accelerations[i + wheels_per_side_];
      msg.error.effort[i + wheels_per_side_] =
        msg.desired.effort[i + wheels_per_side_] - msg.actual.effort[i + wheels_per_side_];

      // Save previous velocities to compute acceleration
      vel_left_actual_previous_[i] = vel_left_actual;
      vel_right_actual_previous_[i] = registered_right_wheel_handles_[i].state->get_velocity();
      vel_left_desired_previous_ = vel_left_desired;
      vel_right_desired_previous_ = vel_right_desired;
    }

    realtime_wheel_joint_controller_state_publisher_->unlockAndPublish();
  }
}
}  // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_drive_controller::DiffDriveController, controller_interface::ControllerInterface)
