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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace diff_drive_controller
{
using namespace std::chrono_literals;
using CallbackReturn = DiffDriveController::CallbackReturn;
using lifecycle_msgs::msg::State;

DiffDriveController::DiffDriveController()
: controller_interface::ControllerInterface() {}

DiffDriveController::DiffDriveController(
  std::vector<std::string> left_wheel_names,
  std::vector<std::string> right_wheel_names,
  std::vector<std::string> write_op_names)
: controller_interface::ControllerInterface(),
  left_wheel_names_(std::move(left_wheel_names)),
  right_wheel_names_(std::move(right_wheel_names)),
  write_op_names_(std::move(write_op_names))
{}

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

  // with the lifecycle node being initialized, we can declare parameters
  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "left_wheel_names",
    left_wheel_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>(
    "right_wheel_names",
    right_wheel_names_);
  lifecycle_node_->declare_parameter<std::vector<std::string>>("write_op_modes", write_op_names_);

  lifecycle_node_->declare_parameter<double>("wheel_separation", wheel_params_.separation);
  lifecycle_node_->declare_parameter<int>("wheels_per_side", wheel_params_.wheels_per_side);
  lifecycle_node_->declare_parameter<double>("wheel_radius", wheel_params_.radius);
  lifecycle_node_->declare_parameter<double>(
    "wheel_separation_multiplier",
    wheel_params_.separation_multiplier);
  lifecycle_node_->declare_parameter<double>(
    "left_wheel_radius_multiplier",
    wheel_params_.left_radius_multiplier);
  lifecycle_node_->declare_parameter<double>(
    "right_wheel_radius_multiplier",
    wheel_params_.right_radius_multiplier);

  lifecycle_node_->declare_parameter<std::string>("odom_frame_id", odom_params_.odom_frame_id);
  lifecycle_node_->declare_parameter<std::string>("base_frame_id", odom_params_.base_frame_id);
  lifecycle_node_->declare_parameter<std::vector<double>>("pose_covariance_diagonal", {});
  lifecycle_node_->declare_parameter<std::vector<double>>("twist_covariance_diagonal", {});
  lifecycle_node_->declare_parameter<bool>("open_loop", odom_params_.open_loop);
  lifecycle_node_->declare_parameter<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

  lifecycle_node_->declare_parameter<int>("cmd_vel_timeout", cmd_vel_timeout_.count());
  lifecycle_node_->declare_parameter<bool>("publish_limited_velocity", publish_limited_velocity_);
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
  if (lifecycle_node_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::SUCCESS;
  }

  const auto current_time = lifecycle_node_->get_clock()->now();
  double & linear_command = received_velocity_msg_ptr_->twist.linear.x;
  double & angular_command = received_velocity_msg_ptr_->twist.angular.z;

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;
  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  if (odom_params_.open_loop) {
    odometry_.updateOpenLoop(linear_command, angular_command, current_time);
  } else {
    double left_position_mean = 0.0;
    double right_position_mean = 0.0;
    for (size_t index = 0; index < wheels.wheels_per_side; ++index) {
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
    left_position_mean /= wheels.wheels_per_side;
    right_position_mean /= wheels.wheels_per_side;

    odometry_.update(left_position_mean, right_position_mean, current_time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

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

  if (odom_params_.enable_odom_tf && odometry_transform_publisher_->is_activated() &&
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

  const auto dt = current_time - received_velocity_msg_ptr_->header.stamp;

  // Brake if cmd_vel has timeout
  if (dt > cmd_vel_timeout_) {
    linear_command = 0.0;
    angular_command = 0.0;
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x,
    update_dt.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, update_dt.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(*received_velocity_msg_ptr_);

  //    Publish limited velocity
  if (publish_limited_velocity_ && limited_velocity_publisher_->is_activated() &&
    realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist.linear.x = linear_command;
    limited_velocity_command.twist.angular.z = angular_command;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  if (received_velocity_msg_ptr_ == nullptr) {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // Compute wheels velocities:
  const double velocity_left = (linear_command - angular_command * wheel_separation / 2.0) /
    left_wheel_radius;
  const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) /
    right_wheel_radius;

  // Set wheels velocities:
  for (size_t index = 0; index < wheels.wheels_per_side; ++index) {
    registered_left_wheel_handles_[index].command->set_cmd(velocity_left);
    registered_right_wheel_handles_[index].command->set_cmd(velocity_right);
  }

  set_op_mode(hardware_interface::OperationMode::ACTIVE);
  return controller_interface::return_type::SUCCESS;
}

CallbackReturn DiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = lifecycle_node_->get_logger();

  // update parameters
  left_wheel_names_ = lifecycle_node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = lifecycle_node_->get_parameter("right_wheel_names").as_string_array();
  write_op_names_ = lifecycle_node_->get_parameter("write_op_modes").as_string_array();

  wheel_params_.separation = lifecycle_node_->get_parameter("wheel_separation").as_double();
  wheel_params_.wheels_per_side =
    static_cast<size_t>(lifecycle_node_->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = lifecycle_node_->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    lifecycle_node_->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier = lifecycle_node_->get_parameter(
    "left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier = lifecycle_node_->get_parameter(
    "right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(
    lifecycle_node_->get_parameter(
      "velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = lifecycle_node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = lifecycle_node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = lifecycle_node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(),
    odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal =
    lifecycle_node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(),
    twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = lifecycle_node_->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = lifecycle_node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ =
    std::chrono::milliseconds{lifecycle_node_->get_parameter("cmd_vel_timeout").as_int()};
  publish_limited_velocity_ = lifecycle_node_->get_parameter("publish_limited_velocity").as_bool();

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

  if (left_wheel_names_.size() != right_wheel_names_.size()) {
    RCLCPP_ERROR(
      logger,
      "The number of left wheels [%d] and the number of right wheels [%d] are different",
      left_wheel_names_.size(),
      right_wheel_names_.size());
    return CallbackReturn::ERROR;
  }

  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  if (auto robot_hardware = robot_hardware_.lock()) {
    const auto left_result =
      configure_side("left", left_wheel_names_, registered_left_wheel_handles_, *robot_hardware);
    const auto right_result =
      configure_side("right", right_wheel_names_, registered_right_wheel_handles_, *robot_hardware);

    if (left_result == CallbackReturn::FAILURE || right_result == CallbackReturn::FAILURE) {
      return CallbackReturn::FAILURE;
    }

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

  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty() ||
    registered_operation_mode_handles_.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Either left wheel handles, right wheel handles, or operation modes are non existant");
    return CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheel_params_.wheels_per_side = registered_left_wheel_handles_.size();

  if (publish_limited_velocity_) {
    limited_velocity_publisher_ =
      lifecycle_node_->create_publisher<Twist>(
      DEFAULT_COMMAND_OUT_TOPIC,
      rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  received_velocity_msg_ptr_ = std::make_shared<Twist>();

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(*received_velocity_msg_ptr_);
  previous_commands_.emplace(*received_velocity_msg_ptr_);

  // initialize command subscriber
  velocity_command_subscriber_ = lifecycle_node_->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](
      const std::shared_ptr<Twist> msg) -> void {
      if (!subscriber_is_active_) {
        RCLCPP_WARN(
          lifecycle_node_->get_logger(),
          "Can't accept new commands. subscriber is inactive");
        return;
      }

      received_velocity_msg_ptr_ = std::move(msg);
    });

  // initialize odometry publisher and messasge
  odometry_publisher_ =
    lifecycle_node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC,
    rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<
    realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(
    rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ =
    lifecycle_node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC,
    rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

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

  RCLCPP_INFO(
    lifecycle_node_->get_logger(), "Lifecycle subscriber and publisher are currently active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  odometry_transform_publisher_->on_deactivate();
  odometry_publisher_->on_deactivate();
  if (publish_limited_velocity_) {
    limited_velocity_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_ = std::make_shared<Twist>();
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

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_operation_mode_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  received_velocity_msg_ptr_.reset();
  is_halted = false;
  return true;
}

CallbackReturn DiffDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
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
  const std::string & side,
  const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles,
  hardware_interface::RobotHardware & robot_hardware)
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
}  // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_drive_controller::DiffDriveController,
  controller_interface::ControllerInterface)
