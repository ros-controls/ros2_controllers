// Copyright 2025 ros2_control development team
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

#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "controller_interface/tf_prefix.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

// Size of one side of the 6x6 odometry covariance matrices
constexpr std::size_t NUM_DIMENSIONS = 6;

// Commands smaller than this are treated as "stop"
constexpr double STOP_EPSILON = 1e-6;

// A wheel that is not yet pointing in its target direction only pushes the robot that way with
// cos(steering_error) of its speed, the rest pushes sideways. Up to 30 degrees of error that loss
// is small (cos(30 deg) = 0.87), so the wheel keeps full speed. Above that the wheel speed is
// scaled by cos(steering_error) / cos(30 deg), which is 1.0 at 30 degrees, so there is no jump.
constexpr double FULL_SPEED_STEERING_ERROR = M_PI / 6.0;  // 30 degrees

// Close to 90 degrees of error cos() goes to zero. Keep a small minimum speed factor so the
// wheel does not stop completely while the steering is still turning.
constexpr double MIN_STEERING_SPEED_FACTOR = 0.01;

const std::array<const char *, 4> MODULE_LABELS = {
  {"front_left", "front_right", "rear_left", "rear_right"}};

}  // namespace

namespace swerve_drive_controller
{

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

SwerveModule::SwerveModule(
  hardware_interface::LoanedCommandInterface & wheel_velocity_command,
  const hardware_interface::LoanedStateInterface & wheel_velocity_state,
  hardware_interface::LoanedCommandInterface & steering_position_command,
  const hardware_interface::LoanedStateInterface & steering_position_state)
: wheel_velocity_command_(wheel_velocity_command),
  wheel_velocity_state_(wheel_velocity_state),
  steering_position_command_(steering_position_command),
  steering_position_state_(steering_position_state)
{
}

bool SwerveModule::set_command(double wheel_velocity, double steering_angle)
{
  const bool wheel_ok = wheel_velocity_command_.get().set_value(wheel_velocity);
  const bool steering_ok = steering_position_command_.get().set_value(steering_angle);
  return wheel_ok && steering_ok;
}

double SwerveModule::get_wheel_velocity() const
{
  return wheel_velocity_state_.get().get_optional().value();
}

double SwerveModule::get_steering_angle() const
{
  return steering_position_state_.get().get_optional().value();
}

SwerveController::SwerveController() : controller_interface::ChainableControllerInterface() {}

CallbackReturn SwerveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    swerve_drive_kinematics_.calculate_wheel_position(
      params_.wheelbase, params_.trackwidth, params_.offset[0], params_.offset[1]);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.front_left_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.front_right_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.rear_left_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.rear_right_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.front_left_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.front_right_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.rear_left_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.rear_right_axle_joint + "/" + HW_IF_POSITION);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.front_left_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.front_right_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.rear_left_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.rear_right_wheel_joint + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.front_left_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.front_right_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.rear_left_axle_joint + "/" + HW_IF_POSITION);
  conf_names.push_back(params_.rear_right_axle_joint + "/" + HW_IF_POSITION);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  try
  {
    wheel_joint_names_ = {
      params_.front_left_wheel_joint, params_.front_right_wheel_joint,
      params_.rear_left_wheel_joint, params_.rear_right_wheel_joint};
    steering_joint_names_ = {
      params_.front_left_axle_joint, params_.front_right_axle_joint, params_.rear_left_axle_joint,
      params_.rear_right_axle_joint};
    max_wheel_speeds_ = {
      {params_.front_left_velocity_threshold, params_.front_right_velocity_threshold,
       params_.rear_left_velocity_threshold, params_.rear_right_velocity_threshold}};

    for (std::size_t i = 0; i < NUM_MODULES; i++)
    {
      if (wheel_joint_names_[i].empty())
      {
        RCLCPP_ERROR(logger, "%s_wheel_joint is not set", MODULE_LABELS[i]);
        return CallbackReturn::ERROR;
      }
      if (steering_joint_names_[i].empty())
      {
        RCLCPP_ERROR(logger, "%s_axle_joint is not set", MODULE_LABELS[i]);
        return CallbackReturn::ERROR;
      }
    }

    // Reserve here so that on_activate does not need to grow the vector
    modules_.reserve(NUM_MODULES);

    cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

    reset();

    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<TwistStamped> msg) -> void
      {
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_.try_set(*msg);
      });

    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    // An empty tf_frame_prefix means: use the controller's namespace ("~")
    std::string tf_frame_prefix = params_.tf_frame_prefix;
    if (tf_frame_prefix.empty())
    {
      tf_frame_prefix = "~";
    }
    const std::string tf_prefix =
      controller_interface::resolve_tf_prefix(tf_frame_prefix, get_node()->get_namespace());

    const auto odom_frame_id = tf_prefix + params_.odom;
    const auto base_frame_id = tf_prefix + params_.base_footprint;
    odometry_message_.header.frame_id = odom_frame_id;
    odometry_message_.child_frame_id = base_frame_id;

    odometry_message_.twist =
      geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    for (std::size_t index = 0; index < NUM_DIMENSIONS; ++index)
    {
      const std::size_t diagonal_index = NUM_DIMENSIONS * index + index;
      odometry_message_.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
      odometry_message_.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
    }

    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);

    odometry_transform_message_.transforms.resize(1);
    odometry_transform_message_.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message_.transforms.front().child_frame_id = base_frame_id;

    previous_update_timestamp_ = get_node()->get_clock()->now();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger, "EXCEPTION DURING on_configure: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> wheel_commands;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> wheel_states;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_commands;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_states;
  wheel_commands.reserve(NUM_MODULES);
  wheel_states.reserve(NUM_MODULES);
  steering_commands.reserve(NUM_MODULES);
  steering_states.reserve(NUM_MODULES);

  const bool found_all =
    controller_interface::get_ordered_interfaces(
      command_interfaces_, wheel_joint_names_, HW_IF_VELOCITY, wheel_commands) &&
    controller_interface::get_ordered_interfaces(
      state_interfaces_, wheel_joint_names_, HW_IF_VELOCITY, wheel_states) &&
    controller_interface::get_ordered_interfaces(
      command_interfaces_, steering_joint_names_, HW_IF_POSITION, steering_commands) &&
    controller_interface::get_ordered_interfaces(
      state_interfaces_, steering_joint_names_, HW_IF_POSITION, steering_states);
  if (!found_all)
  {
    RCLCPP_ERROR(
      logger,
      "Could not find all wheel velocity and steering position interfaces. Expected %zu "
      "wheel and %zu steering joints.",
      NUM_MODULES, NUM_MODULES);
    return CallbackReturn::ERROR;
  }

  modules_.clear();
  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    modules_.emplace_back(
      wheel_commands[i].get(), wheel_states[i].get(), steering_commands[i].get(),
      steering_states[i].get());

    if (!modules_[i].set_command(0.0, 0.0))
    {
      RCLCPP_ERROR(logger, "Failed to set initial command for %s module", MODULE_LABELS[i]);
      modules_.clear();
      return CallbackReturn::ERROR;
    }
    previous_steering_angles_[i] = modules_[i].get_steering_angle();
  }

  RCLCPP_INFO(logger, "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update_reference_from_subscribers(
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
    for (auto & reference_interface : ordered_exported_reference_interfaces_)
    {
      if (!reference_interface->set_value(0.0))
      {
        RCLCPP_WARN_THROTTLE(
          logger, *get_node()->get_clock(), 1000, "Failed to reset reference interface %s",
          reference_interface->get_name().c_str());
      }
    }
  }
  else if (
    std::isfinite(command_msg_.twist.linear.x) && std::isfinite(command_msg_.twist.linear.y) &&
    std::isfinite(command_msg_.twist.angular.z))
  {
    const std::array<double, 3> new_reference = {
      {command_msg_.twist.linear.x, command_msg_.twist.linear.y, command_msg_.twist.angular.z}};
    for (std::size_t i = 0; i < new_reference.size(); i++)
    {
      if (!ordered_exported_reference_interfaces_[i]->set_value(new_reference[i]))
      {
        RCLCPP_WARN_THROTTLE(
          logger, *get_node()->get_clock(), 1000, "Failed to set reference interface %s",
          ordered_exported_reference_interfaces_[i]->get_name().c_str());
      }
    }
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(),
      static_cast<rcutils_duration_value_t>(cmd_vel_timeout_.nanoseconds() / 1000000),
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type SwerveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();

  // The modules only exist while the controller is active
  if (get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double linear_x_cmd =
    ordered_exported_reference_interfaces_[0]->get_optional<double>().value_or(nan);
  const double linear_y_cmd =
    ordered_exported_reference_interfaces_[1]->get_optional<double>().value_or(nan);
  const double angular_cmd =
    ordered_exported_reference_interfaces_[2]->get_optional<double>().value_or(nan);

  if (!std::isfinite(linear_x_cmd) || !std::isfinite(linear_y_cmd) || !std::isfinite(angular_cmd))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  auto wheel_command = swerve_drive_kinematics_.compute_wheel_commands(
    linear_x_cmd, linear_y_cmd, angular_cmd, params_.wheel_radius);

  std::array<double, NUM_MODULES> current_steering_angles{};
  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    current_steering_angles[i] = modules_[i].get_steering_angle();
  }

  wheel_command =
    swerve_drive_kinematics_.optimize_wheel_commands(wheel_command, current_steering_angles);

  // If any wheel is too fast, slow all wheels down by the same factor,
  // so the robot keeps moving in the commanded direction.
  double velocity_limit_scale = 1.0;
  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    const double wheel_speed = std::abs(wheel_command[i].drive_velocity);
    if (wheel_speed > max_wheel_speeds_[i])
    {
      velocity_limit_scale = std::min(velocity_limit_scale, max_wheel_speeds_[i] / wheel_speed);
    }
  }

  // Slow down wheels that are still turning towards their target direction
  const double cos_full_speed_steering_error = std::cos(FULL_SPEED_STEERING_ERROR);
  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    const double steering_error = std::abs(
      angles::shortest_angular_distance(
        current_steering_angles[i], wheel_command[i].steering_angle));

    double steering_scale = 1.0;
    if (steering_error > FULL_SPEED_STEERING_ERROR)
    {
      steering_scale = std::max(std::cos(steering_error), MIN_STEERING_SPEED_FACTOR) /
                       cos_full_speed_steering_error;
    }

    wheel_command[i].drive_velocity *= velocity_limit_scale * steering_scale;
    wheel_command[i].drive_angular_velocity *= velocity_limit_scale * steering_scale;
  }

  // When the robot is told to stop, keep the wheels pointing where they are
  const bool is_stop = (std::fabs(linear_x_cmd) < STOP_EPSILON) &&
                       (std::fabs(linear_y_cmd) < STOP_EPSILON) &&
                       (std::fabs(angular_cmd) < STOP_EPSILON);

  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    if (!is_stop)
    {
      previous_steering_angles_[i] = wheel_command[i].steering_angle;
    }

    if (!modules_[i].set_command(
          wheel_command[i].drive_angular_velocity, previous_steering_angles_[i]))
    {
      RCLCPP_WARN_THROTTLE(
        logger, *get_node()->get_clock(), 1000, "Failed to write command for %s module",
        MODULE_LABELS[i]);
    }
  }

  const auto update_dt = time - previous_update_timestamp_;
  previous_update_timestamp_ = time;

  std::array<double, NUM_MODULES> wheel_speeds{};  // m/s
  std::array<double, NUM_MODULES> steering_angles{};
  for (std::size_t i = 0; i < NUM_MODULES; i++)
  {
    if (params_.open_loop)
    {
      wheel_speeds[i] = wheel_command[i].drive_velocity;
      steering_angles[i] = wheel_command[i].steering_angle;
    }
    else
    {
      wheel_speeds[i] = modules_[i].get_wheel_velocity() * params_.wheel_radius;
      steering_angles[i] = modules_[i].get_steering_angle();
    }
  }
  odometry_ =
    swerve_drive_kinematics_.update_odometry(wheel_speeds, steering_angles, update_dt.seconds());

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.theta);

  if (realtime_odometry_publisher_)
  {
    odometry_message_.header.stamp = time;
    odometry_message_.pose.pose.position.x = odometry_.x;
    odometry_message_.pose.pose.position.y = odometry_.y;
    odometry_message_.pose.pose.orientation.x = orientation.x();
    odometry_message_.pose.pose.orientation.y = orientation.y();
    odometry_message_.pose.pose.orientation.z = orientation.z();
    odometry_message_.pose.pose.orientation.w = orientation.w();
    odometry_message_.twist.twist.linear.x = odometry_.vx;
    odometry_message_.twist.twist.linear.y = odometry_.vy;
    odometry_message_.twist.twist.angular.z = odometry_.wz;
    realtime_odometry_publisher_->try_publish(odometry_message_);
  }

  if (params_.enable_odom_tf && realtime_odometry_transform_publisher_)
  {
    auto & transform = odometry_transform_message_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = odometry_.x;
    transform.transform.translation.y = odometry_.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->try_publish(odometry_transform_message_);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  halt();
  modules_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
{
  reset();
  return CallbackReturn::SUCCESS;
}

void SwerveController::reset()
{
  velocity_command_subscriber_.reset();

  TwistStamped zero_twist;
  zero_twist.header.stamp = get_node()->get_clock()->now();
  command_msg_ = zero_twist;
  received_velocity_msg_.set(zero_twist);
}

void SwerveController::halt()
{
  for (std::size_t i = 0; i < modules_.size(); i++)
  {
    if (!modules_[i].set_command(0.0, 0.0))
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to stop %s module", MODULE_LABELS[i]);
    }
  }
}

bool SwerveController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface::SharedPtr>
SwerveController::on_export_reference_interfaces_list()
{
  const std::vector<std::string> reference_interface_names = {
    "/linear/x", "/linear/y", "/angular/z"};

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;
  reference_interfaces.reserve(reference_interface_names.size());

  for (const auto & name : reference_interface_names)
  {
    auto reference_interface = std::make_shared<hardware_interface::CommandInterface>(
      get_node()->get_name() + name, hardware_interface::HW_IF_VELOCITY);
    if (!reference_interface->set_value(std::numeric_limits<double>::quiet_NaN()))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to initialize reference interface %s",
        reference_interface->get_name().c_str());
    }
    reference_interfaces.push_back(reference_interface);
  }

  return reference_interfaces;
}

}  // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_drive_controller::SwerveController, controller_interface::ChainableControllerInterface)
