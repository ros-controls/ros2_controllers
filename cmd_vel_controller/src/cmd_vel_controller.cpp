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

#include "cmd_vel_controller/cmd_vel_controller.hpp"

#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_IN_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/cmd_vel_out";
}  // namespace

namespace cmd_vel_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using lifecycle_msgs::msg::State;

CmdVelController::CmdVelController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn CmdVelController::on_init()
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

controller_interface::CallbackReturn CmdVelController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // Update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

  // Allocate reference interfaces if needed
  const int nr_ref_itfs = 6;
  reference_interfaces_.resize(nr_ref_itfs, std::numeric_limits<double>::quiet_NaN());

  // Initialize speed limiters
  velocity_limiters_.resize(nr_ref_itfs);
  velocity_limiters_[0] = control_toolbox::RateLimiter<double>(
    params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.max_acceleration_reverse, params_.linear.x.max_acceleration,
    params_.linear.x.max_deceleration, params_.linear.x.max_deceleration_reverse,
    params_.linear.x.min_jerk, params_.linear.x.max_jerk);
  velocity_limiters_[1] = control_toolbox::RateLimiter<double>(
    params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.max_acceleration_reverse, params_.linear.y.max_acceleration,
    params_.linear.y.max_deceleration, params_.linear.y.max_deceleration_reverse,
    params_.linear.y.min_jerk, params_.linear.y.max_jerk);
  velocity_limiters_[2] = control_toolbox::RateLimiter<double>(
    params_.linear.z.min_velocity, params_.linear.z.max_velocity,
    params_.linear.z.max_acceleration_reverse, params_.linear.z.max_acceleration,
    params_.linear.z.max_deceleration, params_.linear.z.max_deceleration_reverse,
    params_.linear.z.min_jerk, params_.linear.z.max_jerk);
  velocity_limiters_[3] = control_toolbox::RateLimiter<double>(
    params_.angular.x.min_velocity, params_.angular.x.max_velocity,
    params_.angular.x.max_acceleration_reverse, params_.angular.x.max_acceleration,
    params_.angular.x.max_deceleration, params_.angular.x.max_deceleration_reverse,
    params_.angular.x.min_jerk, params_.angular.x.max_jerk);
  velocity_limiters_[4] = control_toolbox::RateLimiter<double>(
    params_.angular.y.min_velocity, params_.angular.y.max_velocity,
    params_.angular.y.max_acceleration_reverse, params_.angular.y.max_acceleration,
    params_.angular.y.max_deceleration, params_.angular.y.max_deceleration_reverse,
    params_.angular.y.min_jerk, params_.angular.y.max_jerk);
  velocity_limiters_[5] = control_toolbox::RateLimiter<double>(
    params_.angular.z.min_velocity, params_.angular.z.max_velocity,
    params_.angular.z.max_acceleration_reverse, params_.angular.z.max_acceleration,
    params_.angular.z.max_deceleration, params_.angular.z.max_deceleration_reverse,
    params_.angular.z.min_jerk, params_.angular.z.max_jerk);

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

  // Initialize cmd_vel publisher and message
  cmd_vel_publisher_ = get_node()->create_publisher<TwistStamped>(
    DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_cmd_vel_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<TwistStamped>>(cmd_vel_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration CmdVelController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const std::string & interface_name : params_.interface_names)
  {
    if (interface_name != "")
    {
      conf_names.push_back(interface_name);
    }
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration CmdVelController::state_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn CmdVelController::on_activate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CmdVelController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  reset_buffers();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CmdVelController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  auto logger = get_node()->get_logger();

  auto current_ref_op = received_velocity_msg_.try_get();
  if (current_ref_op.has_value())
  {
    received_command_msg_ = current_ref_op.value();
  }
  cmd_vel_msg_.header.frame_id = received_command_msg_.header.frame_id;

  const auto age_of_last_command = time - received_command_msg_.header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
    reference_interfaces_[2] = 0.0;
    reference_interfaces_[3] = 0.0;
    reference_interfaces_[4] = 0.0;
    reference_interfaces_[5] = 0.0;
  }
  else if (
    std::isfinite(received_command_msg_.twist.linear.x) &&
    std::isfinite(received_command_msg_.twist.linear.y) &&
    std::isfinite(received_command_msg_.twist.linear.z) &&
    std::isfinite(received_command_msg_.twist.angular.x) &&
    std::isfinite(received_command_msg_.twist.angular.y) &&
    std::isfinite(received_command_msg_.twist.angular.z))
  {
    reference_interfaces_[0] = received_command_msg_.twist.linear.x;
    reference_interfaces_[1] = received_command_msg_.twist.linear.y;
    reference_interfaces_[2] = received_command_msg_.twist.linear.z;
    reference_interfaces_[3] = received_command_msg_.twist.angular.x;
    reference_interfaces_[4] = received_command_msg_.twist.angular.y;
    reference_interfaces_[5] = received_command_msg_.twist.angular.z;
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(), cmd_vel_timeout_.seconds() * 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type CmdVelController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  rclcpp::Logger logger = get_node()->get_logger();

  // Update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  for (size_t i = 0; i < 6; ++i)
  {
    if (!std::isfinite(reference_interfaces_[i]))
    {
      return controller_interface::return_type::OK;
    }
  }

  std::vector<double> command(reference_interfaces_.begin(), reference_interfaces_.begin() + 6);

  // Apply rate limiting to the command and record the smallest limiting factor
  double limiting_factor = 1.0;
  for (size_t i = 0; i < 6; ++i)
  {
    double new_limiting_factor = velocity_limiters_[i].limit(
      command[i], previous_two_commands_.back()[i], previous_two_commands_.front()[i],
      period.seconds());
    if (new_limiting_factor < limiting_factor)
    {
      limiting_factor = new_limiting_factor;
    }
  }
  // Limit all velocities with the smallest limiting factor
  for (size_t i = 0; i < 6; ++i)
  {
    command[i] = reference_interfaces_[i] * limiting_factor;
  }
  previous_two_commands_.pop();
  previous_two_commands_.push(command);

  for (size_t i = 0; i < 6; ++i)
  {
    bool set_command_result = true;
    if (params_.interface_names[i] != "")
    {
      // Get command handle``
      const std::string interface_name = params_.interface_names[i];
      const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&interface_name](const auto & interface)
        { return interface.get_name() == interface_name; });
      if (command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain command handle for %s", interface_name.c_str());
        return controller_interface::return_type::ERROR;
      }
      // Write command
      set_command_result &= command_handle->set_value(command[i]);
    }
    RCLCPP_DEBUG_EXPRESSION(
      logger, !set_command_result, "Unable to set the command to one of the command handles!");
  }

  if (params_.publish_cmd_vel)
  {
    cmd_vel_msg_.header.stamp = get_node()->now();
    cmd_vel_msg_.twist.linear.x = command[0];
    cmd_vel_msg_.twist.linear.y = command[1];
    cmd_vel_msg_.twist.linear.z = command[2];
    cmd_vel_msg_.twist.angular.x = command[3];
    cmd_vel_msg_.twist.angular.y = command[4];
    cmd_vel_msg_.twist.angular.z = command[5];
    realtime_cmd_vel_publisher_->try_publish(cmd_vel_msg_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CmdVelController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CmdVelController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool CmdVelController::reset()
{
  reset_buffers();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  return true;
}

void CmdVelController::reset_buffers()
{
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  // Empty out the old queue. Fill with zeros (not NaN) to catch early accelerations.
  std::queue<std::vector<double>> empty;
  std::swap(previous_two_commands_, empty);
  previous_two_commands_.push(std::vector<double>(6, 0.0));
  previous_two_commands_.push(std::vector<double>(6, 0.0));
  // Fill RealtimeBox with NaNs so it will contain a known value
  // but still indicate that no command has yet been sent.
  received_command_msg_.header.stamp = get_node()->now();
  received_command_msg_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  received_command_msg_.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  received_command_msg_.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  received_command_msg_.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  received_command_msg_.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  received_command_msg_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  received_velocity_msg_.set(received_command_msg_);
}

bool CmdVelController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface> CmdVelController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  std::vector<std::string> reference_interface_names = {"/linear/x",  "/linear/y",  "/linear/z",
                                                        "/angular/x", "/angular/y", "/angular/z"};

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name() + reference_interface_names[i], hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[i]));
  }

  return reference_interfaces;
}
}  // namespace cmd_vel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cmd_vel_controller::CmdVelController, controller_interface::ChainableControllerInterface)
