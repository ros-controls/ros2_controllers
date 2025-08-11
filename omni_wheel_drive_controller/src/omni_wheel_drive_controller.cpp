// Copyright 2025 Aarav Gupta
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

#include "omni_wheel_drive_controller/omni_wheel_drive_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_IN_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace omni_wheel_drive_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

OmniWheelDriveController::OmniWheelDriveController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn OmniWheelDriveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  params_ = param_listener_->get_params();

  if (params_.wheel_names.size() < 3)
  {
    RCLCPP_ERROR(
      logger,
      "The number of wheels is less than 3. If there are more wheels please pass them all to the "
      "controller or use a different one for your wheel configuration.");
    return controller_interface::CallbackReturn::ERROR;
  }

  odometry_.setParams(
    params_.robot_radius, params_.wheel_radius, params_.wheel_offset, params_.wheel_names.size());

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

  // Allocate reference interfaces if needed
  const int nr_ref_itfs = 3;
  reference_interfaces_.resize(nr_ref_itfs, std::numeric_limits<double>::quiet_NaN());

  // Reset the controller
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize velocity command subscriber and define its callback function
  velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
    DEFAULT_COMMAND_IN_TOPIC, rclcpp::SystemDefaultsQoS(),
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

  // Initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    // Make sure prefix does not start with '/' and always ends with '/'
    if (tf_prefix.back() != '/')
    {
      tf_prefix = tf_prefix + "/";
    }
    if (tf_prefix.front() == '/')
    {
      tf_prefix.erase(0, 1);
    }
  }
  const std::string odom_frame_id = tf_prefix + params_.odom_frame_id;
  const std::string base_frame_id = tf_prefix + params_.base_frame_id;

  odometry_message_.header.frame_id = odom_frame_id;
  odometry_message_.child_frame_id = base_frame_id;

  // Initialize odom values zeros
  odometry_message_.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message_.pose.covariance[diagonal_index] = params_.diagonal_covariance.pose[index];
    odometry_message_.twist.covariance[diagonal_index] = params_.diagonal_covariance.twist[index];
  }

  // Initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  transform_.header.frame_id = odom_frame_id;
  transform_.child_frame_id = base_frame_id;

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration OmniWheelDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration OmniWheelDriveController::state_interface_configuration() const
{
  if (params_.open_loop)
  {
    return {interface_configuration_type::NONE, {}};
  }

  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn OmniWheelDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Register wheel handles and check if any errors happen
  if (
    configure_wheel_handles(params_.wheel_names, registered_wheel_handles_) ==
    controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  halt();
  reset_buffers();
  registered_wheel_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

const char * OmniWheelDriveController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::return_type OmniWheelDriveController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
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
    reference_interfaces_[2] = 0.0;
  }
  else if (
    std::isfinite(command_msg_.twist.linear.x) && std::isfinite(command_msg_.twist.linear.y) &&
    std::isfinite(command_msg_.twist.angular.z))
  {
    reference_interfaces_[0] = command_msg_.twist.linear.x;
    reference_interfaces_[1] = command_msg_.twist.linear.y;
    reference_interfaces_[2] = command_msg_.twist.angular.z;
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(),
      static_cast<rcutils_duration_value_t>(cmd_vel_timeout_.seconds() * 1000),
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type OmniWheelDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  rclcpp::Logger logger = get_node()->get_logger();

  if (
    !std::isfinite(reference_interfaces_[0]) || !std::isfinite(reference_interfaces_[1]) ||
    !std::isfinite(reference_interfaces_[2]))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  // Update odometry
  bool odometry_updated = false;
  if (params_.open_loop)
  {
    odometry_updated = odometry_.updateOpenLoop(
      reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2], time);
  }
  else
  {
    std::vector<double> wheels_feedback(registered_wheel_handles_.size());  // [rads]
    for (size_t i = 0; i < static_cast<size_t>(registered_wheel_handles_.size()); ++i)
    {
      // Get wheel feedback
      const std::optional<double> wheel_feedback_op =
        registered_wheel_handles_[i].feedback.value().get().get_optional();
      if (!wheel_feedback_op.has_value())
      {
        RCLCPP_DEBUG(logger, "Unable to retrieve data from [%zu] wheel feedback!", i);
        return controller_interface::return_type::OK;
      }
      const double wheel_feedback = wheel_feedback_op.value();

      if (std::isnan(wheel_feedback))
      {
        RCLCPP_ERROR(logger, "The wheel %s is invalid for index [%zu]", feedback_type(), i);
        return controller_interface::return_type::ERROR;
      }

      wheels_feedback[i] = wheel_feedback;
    }
    if (params_.position_feedback)
    {
      odometry_updated = odometry_.updateFromPos(wheels_feedback, time);
    }
    else
    {
      odometry_updated = odometry_.updateFromVel(wheels_feedback, time);
    }
  }

  if (odometry_updated)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    odometry_message_.header.stamp = get_node()->now();
    odometry_message_.pose.pose.position.x = odometry_.getX();
    odometry_message_.pose.pose.position.y = odometry_.getY();
    odometry_message_.pose.pose.orientation.x = orientation.x();
    odometry_message_.pose.pose.orientation.y = orientation.y();
    odometry_message_.pose.pose.orientation.z = orientation.z();
    odometry_message_.pose.pose.orientation.w = orientation.w();
    odometry_message_.twist.twist.linear.x = odometry_.getLinearXVel();
    odometry_message_.twist.twist.linear.y = odometry_.getLinearYVel();
    odometry_message_.twist.twist.angular.z = odometry_.getAngularVel();
    realtime_odometry_publisher_->try_publish(odometry_message_);

    if (params_.enable_odom_tf)
    {
      transform_.header.stamp = get_node()->now();
      transform_.transform.translation.x = odometry_.getX();
      transform_.transform.translation.y = odometry_.getY();
      transform_.transform.rotation.x = orientation.x();
      transform_.transform.rotation.y = orientation.y();
      transform_.transform.rotation.z = orientation.z();
      transform_.transform.rotation.w = orientation.w();
      tf2_msgs::msg::TFMessage odometry_transform_message;
      odometry_transform_message.transforms.push_back(transform_);
      realtime_odometry_transform_publisher_->try_publish(odometry_transform_message);
    }
  }

  compute_and_set_wheel_velocities();

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniWheelDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelDriveController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void OmniWheelDriveController::compute_and_set_wheel_velocities()
{
  bool set_command_result = true;

  double angle_bw_wheels = (2 * M_PI) / static_cast<double>(registered_wheel_handles_.size());
  for (size_t i = 0; i < static_cast<size_t>(registered_wheel_handles_.size()); ++i)
  {
    set_command_result &= registered_wheel_handles_[i].velocity.get().set_value(
      ((std::sin((angle_bw_wheels * static_cast<double>(i)) + params_.wheel_offset) *
        reference_interfaces_[0]) -
       (std::cos((angle_bw_wheels * static_cast<double>(i)) + params_.wheel_offset) *
        reference_interfaces_[1]) -
       (reference_interfaces_[2] * params_.robot_radius)) /
      params_.wheel_radius);
  }

  rclcpp::Logger logger = get_node()->get_logger();
  RCLCPP_DEBUG_EXPRESSION(
    logger, !set_command_result, "Unable to set the command to one of the command handles!");
}

bool OmniWheelDriveController::reset()
{
  odometry_.resetOdometry();

  reset_buffers();

  registered_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  return true;
}

controller_interface::CallbackReturn OmniWheelDriveController::configure_wheel_handles(
  const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles)
{
  rclcpp::Logger logger = get_node()->get_logger();

  // Register handles
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
        WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void OmniWheelDriveController::halt()
{
  bool set_command_result = true;
  for (const WheelHandle & wheel_handle : registered_wheel_handles_)
  {
    set_command_result &= wheel_handle.velocity.get().set_value(0.0);
  }
  rclcpp::Logger logger = get_node()->get_logger();
  RCLCPP_DEBUG_EXPRESSION(
    logger, !set_command_result, "Unable to set the command to one of the command handles!");
}

void OmniWheelDriveController::reset_buffers()
{
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

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

bool OmniWheelDriveController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface>
OmniWheelDriveController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  std::vector<std::string> reference_interface_names = {"/linear/x", "/linear/y", "/angular/z"};

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name() + reference_interface_names[i], hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[i]));
  }

  return reference_interfaces;
}
}  // namespace omni_wheel_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  omni_wheel_drive_controller::OmniWheelDriveController,
  controller_interface::ChainableControllerInterface)
