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

#include "controller_interface/tf_prefix.hpp"
#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace diff_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

DiffDriveController::DiffDriveController() : controller_interface::ChainableControllerInterface() {}

const char * DiffDriveController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn DiffDriveController::on_init()
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
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration DiffDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration DiffDriveController::state_interface_configuration() const
{
  if (params_.open_loop)
  {
    return {interface_configuration_type::NONE, {}};
  }

  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DiffDriveController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();

  auto current_ref_op = received_velocity_msg_.try_get();
  if (current_ref_op.has_value())
  {
    command_msg_ = current_ref_op.value();
  }

  const auto age_of_last_command = time - command_msg_.header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
  }
  else if (
    std::isfinite(command_msg_.twist.linear.x) && std::isfinite(command_msg_.twist.angular.z))
  {
    reference_interfaces_[0] = command_msg_.twist.linear.x;
    reference_interfaces_[1] = command_msg_.twist.angular.z;
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(),
      static_cast<rcutils_duration_value_t>(cmd_vel_timeout_.seconds() * 1000),
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  previous_update_timestamp_ = time;

  return controller_interface::return_type::OK;
}

controller_interface::return_type DiffDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  double linear_command = reference_interfaces_[0];
  double angular_command = reference_interfaces_[1];

  if (!std::isfinite(linear_command) || !std::isfinite(angular_command))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  // Apply (possibly new) multipliers:
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  // Update odometry
  bool odometry_updated = false;
  if (params_.open_loop)
  {
    odometry_updated =
      odometry_.try_update_open_loop(linear_command, angular_command, period.seconds());
  }
  else
  {
    double left_feedback_mean = 0.0;
    double right_feedback_mean = 0.0;
    for (size_t index = 0; index < static_cast<size_t>(wheels_per_side_); ++index)
    {
      const auto left_feedback_op =
        registered_left_wheel_handles_[index].feedback.value().get().get_optional();
      const auto right_feedback_op =
        registered_right_wheel_handles_[index].feedback.value().get().get_optional();

      if (!left_feedback_op.has_value() || !right_feedback_op.has_value())
      {
        RCLCPP_DEBUG(logger, "Unable to retrieve the data from the left or right wheels feedback!");
        return controller_interface::return_type::OK;
      }

      const double left_feedback = left_feedback_op.value();
      const double right_feedback = right_feedback_op.value();

      if (std::isnan(left_feedback) || std::isnan(right_feedback))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel %s is invalid for index [%zu]", feedback_type(),
          index);
        return controller_interface::return_type::ERROR;
      }

      left_feedback_mean += left_feedback;
      right_feedback_mean += right_feedback;
    }
    left_feedback_mean /= static_cast<double>(wheels_per_side_);
    right_feedback_mean /= static_cast<double>(wheels_per_side_);

    if (params_.position_feedback)
    {
      odometry_updated =
        odometry_.update_from_pos(left_feedback_mean, right_feedback_mean, period.seconds());
    }
    else
    {
      odometry_updated =
        odometry_.update_from_vel(left_feedback_mean, right_feedback_mean, period.seconds());
    }
  }

  if (odometry_updated)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    bool should_publish = false;
    try
    {
      if (previous_publish_timestamp_ + publish_period_ <= time)
      {
        previous_publish_timestamp_ += publish_period_;
        should_publish = true;
      }
    }
    catch (const std::runtime_error &)
    {
      // Handle exceptions when the time source changes and initialize publish timestamp
      previous_publish_timestamp_ = time;
      should_publish = true;
    }

    if (should_publish)
    {
      if (realtime_odometry_publisher_)
      {
        odometry_message_.header.stamp = time;
        odometry_message_.pose.pose.position.x = odometry_.getX();
        odometry_message_.pose.pose.position.y = odometry_.getY();
        odometry_message_.pose.pose.orientation.x = orientation.x();
        odometry_message_.pose.pose.orientation.y = orientation.y();
        odometry_message_.pose.pose.orientation.z = orientation.z();
        odometry_message_.pose.pose.orientation.w = orientation.w();
        odometry_message_.twist.twist.linear.x = odometry_.getLinear();
        odometry_message_.twist.twist.angular.z = odometry_.getAngular();
        realtime_odometry_publisher_->try_publish(odometry_message_);
      }

      if (params_.enable_odom_tf && realtime_odometry_transform_publisher_)
      {
        auto & transform = odometry_transform_message_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.getX();
        transform.transform.translation.y = odometry_.getY();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        realtime_odometry_transform_publisher_->try_publish(odometry_transform_message_);
      }
    }
  }

  double & last_linear = previous_two_commands_.back()[0];
  double & second_to_last_linear = previous_two_commands_.front()[0];
  double & last_angular = previous_two_commands_.back()[1];
  double & second_to_last_angular = previous_two_commands_.front()[1];

  limiter_linear_->limit(linear_command, last_linear, second_to_last_linear, period.seconds());
  limiter_angular_->limit(angular_command, last_angular, second_to_last_angular, period.seconds());
  previous_two_commands_.pop();
  previous_two_commands_.push({{linear_command, angular_command}});

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_)
  {
    limited_velocity_message_.header.stamp = time;
    limited_velocity_message_.twist.linear.x = linear_command;
    limited_velocity_message_.twist.linear.y = 0.0;
    limited_velocity_message_.twist.linear.z = 0.0;
    limited_velocity_message_.twist.angular.x = 0.0;
    limited_velocity_message_.twist.angular.y = 0.0;
    limited_velocity_message_.twist.angular.z = angular_command;
    realtime_limited_velocity_publisher_->try_publish(limited_velocity_message_);
  }

  // Compute wheels velocities:
  const double velocity_left =
    (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right =
    (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

  // Set wheels velocities:
  bool set_command_result = true;
  for (size_t index = 0; index < static_cast<size_t>(wheels_per_side_); ++index)
  {
    set_command_result &=
      registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    set_command_result &=
      registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  }

  RCLCPP_DEBUG_EXPRESSION(
    logger, !set_command_result, "Unable to set the command to one of the command handles!");

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DiffDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->try_update_params(params_))
  {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      params_.left_wheel_names.size(), params_.right_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(static_cast<size_t>(params_.velocity_rolling_window_size));

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);
  publish_limited_velocity_ = params_.publish_limited_velocity;

  // Allocate reference interfaces if needed
  const int nr_ref_itfs = 2;
  reference_interfaces_.resize(nr_ref_itfs, std::numeric_limits<double>::quiet_NaN());

  limiter_linear_ = std::make_unique<SpeedLimiter>(
    params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.max_acceleration_reverse, params_.linear.x.max_acceleration,
    params_.linear.x.max_deceleration, params_.linear.x.max_deceleration_reverse,
    params_.linear.x.min_jerk, params_.linear.x.max_jerk);

  limiter_angular_ = std::make_unique<SpeedLimiter>(
    params_.angular.z.min_velocity, params_.angular.z.max_velocity,
    params_.angular.z.max_acceleration_reverse, params_.angular.z.max_acceleration,
    params_.angular.z.max_deceleration, params_.angular.z.max_deceleration_reverse,
    params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheels_per_side_ = static_cast<int>(params_.left_wheel_names.size());

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = get_node()->create_publisher<TwistStamped>(
      DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<TwistStamped>>(
        limited_velocity_publisher_);
  }

  // initialize command subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<TwistStamped> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->now();
      }

      const auto current_time_diff = get_node()->now() - msg->header.stamp;

      if (
        cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
        current_time_diff < cmd_vel_timeout_)
      {
        received_velocity_msg_.set(*msg);
      }
      else
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring the received message (timestamp %.10f) because it is older than "
          "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
          rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
          cmd_vel_timeout_.seconds());
      }
    });

  // deprecation warning if tf_frame_prefix_enable set to non-default value
  const bool default_tf_frame_prefix_enable = true;
  if (params_.tf_frame_prefix_enable != default_tf_frame_prefix_enable)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter 'tf_frame_prefix_enable' is DEPRECATED and set to a non-default value (%s). "
      "Please migrate to 'tf_frame_prefix'.",
      params_.tf_frame_prefix_enable ? "true" : "false");
  }

  // initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // resolve prefix: substitute tilde (~) with the namespace if contains and normalize slashes (/)
  const auto tf_prefix =
    controller_interface::resolve_tf_prefix(params_.tf_frame_prefix, get_node()->get_namespace());

  // prepend resolved TF prefix to frame ids
  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  odometry_message_.header.frame_id = odom_frame_id;
  odometry_message_.child_frame_id = base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message_.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message_.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message_.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  odometry_transform_message_.transforms.resize(1);
  odometry_transform_message_.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message_.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto left_result =
    configure_side("left", params_.left_wheel_names, registered_left_wheel_handles_);
  const auto right_result =
    configure_side("right", params_.right_wheel_names, registered_right_wheel_handles_);

  if (
    left_result == controller_interface::CallbackReturn::ERROR ||
    right_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either left wheel interfaces, right wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  halt();
  reset_buffers();
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool DiffDriveController::reset()
{
  odometry_.resetOdometry();

  reset_buffers();

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  return true;
}

void DiffDriveController::reset_buffers()
{
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  // Empty out the old queue. Fill with zeros (not NaN) to catch early accelerations.
  std::queue<std::array<double, 2>> empty;
  std::swap(previous_two_commands_, empty);
  previous_two_commands_.push({{0.0, 0.0}});
  previous_two_commands_.push({{0.0, 0.0}});

  // Fill RealtimeBox with NaNs so it will contain a known value
  // but still indicate that no command has yet been sent.
  command_msg_.header.stamp = get_node()->now();
  command_msg_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  command_msg_.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  command_msg_.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  command_msg_.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  command_msg_.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  command_msg_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  received_velocity_msg_.set(command_msg_);
}

void DiffDriveController::halt()
{
  auto logger = get_node()->get_logger();
  const auto halt_wheels = [&](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      if (!wheel_handle.velocity.get().set_value(0.0))
      {
        RCLCPP_WARN(logger, "Failed to set wheel velocity to value 0.0");
      }
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}

controller_interface::CallbackReturn DiffDriveController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.open_loop)
    {
      registered_handles.emplace_back(WheelHandle{std::nullopt, std::ref(*command_handle)});
    }
    else
    {
      const auto interface_name = feedback_type();
      const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_name, &interface_name](const auto & interface)
        {
          return interface.get_prefix_name() == wheel_name &&
                 interface.get_interface_name() == interface_name;
        });

      if (state_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(
        WheelHandle{{std::ref(*state_handle)}, std::ref(*command_handle)});
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool DiffDriveController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface>
DiffDriveController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/linear"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[0]));

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/angular"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[1]));

  return reference_interfaces;
}

}  // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_drive_controller::DiffDriveController, controller_interface::ChainableControllerInterface)
