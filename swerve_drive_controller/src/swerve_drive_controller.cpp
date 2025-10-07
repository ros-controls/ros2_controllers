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

#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

}  // namespace

namespace swerve_drive_controller
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

Wheel::Wheel(
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity,
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback, std::string name)
: velocity_(velocity), feedback_(feedback), name_(std::move(name))
{
}

void Wheel::set_velocity(double velocity) { velocity_.get().set_value(velocity); }

double Wheel::get_feedback() { return Wheel::feedback_.get().get_optional().value(); }

Axle::Axle(
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> position,
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback, std::string name)
: position_(position), feedback_(feedback), name_(std::move(name))
{
}

void Axle::set_position(double position) { position_.get().set_value(position); }

double Axle::get_feedback() { return Axle::feedback_.get().get_optional().value(); }

SwerveController::SwerveController()

: controller_interface::ControllerInterface()
{
  auto zero_twist = std::make_shared<TwistStamped>();
  zero_twist->header.stamp = rclcpp::Time(0);
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.writeFromNonRT(zero_twist);
}

CallbackReturn SwerveController::on_init()
{
  auto node = get_node();
  if (!node)
  {
    RCLCPP_ERROR(node->get_logger(), "Node is null in on_init");
    return controller_interface::CallbackReturn::ERROR;
  }
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    swerveDriveKinematics_.calculate_wheel_position(
      params_.wheelbase, params_.trackwidth, params_.offset[0], params_.offset[1]);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
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
    for (std::size_t i = 0; i < 6; ++i)
    {
      params_.pose_covariance_diagonal[i] = 0.01;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
      params_.twist_covariance_diagonal[i] = 0.01;
    }
    if (params_.front_left_wheel_joint.empty())
    {
      RCLCPP_ERROR(logger, "front_left_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.front_right_wheel_joint.empty())
    {
      RCLCPP_ERROR(logger, "front_right_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.rear_left_wheel_joint.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.rear_right_wheel_joint.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.front_left_axle_joint.empty())
    {
      RCLCPP_ERROR(logger, "front_left_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.front_right_axle_joint.empty())
    {
      RCLCPP_ERROR(logger, "front_right_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.rear_left_axle_joint.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_axle_joint_name_ is not set");
      return CallbackReturn::ERROR;
    }
    if (params_.rear_right_axle_joint.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_axle_joint_name_ is not set");
      return CallbackReturn::ERROR;
    }
    wheel_joint_names[0] = params_.front_left_wheel_joint;
    wheel_joint_names[1] = params_.front_right_wheel_joint;
    wheel_joint_names[2] = params_.rear_left_wheel_joint;
    wheel_joint_names[3] = params_.rear_right_wheel_joint;
    axle_joint_names[0] = params_.front_left_axle_joint;
    axle_joint_names[1] = params_.front_right_axle_joint;
    axle_joint_names[2] = params_.rear_left_axle_joint;
    axle_joint_names[3] = params_.rear_right_axle_joint;
    cmd_vel_timeout_ =
      std::chrono::milliseconds(static_cast<int>(params_.cmd_vel_timeout * 1000.0));
    if (!reset())
    {
      return CallbackReturn::ERROR;
    }

    TwistStamped empty_twist;
    empty_twist.header.stamp = get_node()->now();
    empty_twist.twist.linear.x = 0.0;
    empty_twist.twist.linear.y = 0.0;
    empty_twist.twist.angular.z = 0.0;
    received_velocity_msg_ptr_.writeFromNonRT(std::make_shared<TwistStamped>(empty_twist));

    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this, logger](const std::shared_ptr<TwistStamped> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(logger, "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            logger,
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.writeFromNonRT(std::move(msg));
      });

    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

    realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    std::string tf_prefix = "";
    tf_prefix = std::string(get_node()->get_namespace());
    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }

    const auto odom_frame_id = tf_prefix + params_.odom;
    const auto base_frame_id = tf_prefix + params_.base_footprint;
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = odom_frame_id;
    odometry_message.child_frame_id = base_frame_id;

    odometry_message.twist =
      geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr std::size_t NUM_DIMENSIONS = 6;
    for (std::size_t index = 0; index < 6; ++index)
    {
      const std::size_t diagonal_index = NUM_DIMENSIONS * index + index;
      odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
      odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
    }
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
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

  wheel_handles_.resize(4);
  for (std::size_t i = 0; i < 4; i++)
  {
    wheel_handles_[i] = get_wheel(command_interfaces_, state_interfaces_, wheel_joint_names[i]);
  }

  axle_handles_.resize(4);
  for (std::size_t i = 0; i < 4; i++)
  {
    axle_handles_[i] = get_axle(command_interfaces_, state_interfaces_, axle_joint_names[i]);
  }

  for (std::size_t i = 0; i < wheel_handles_.size(); ++i)
  {
    if (!wheel_handles_[i])
    {
      RCLCPP_ERROR(logger, "ERROR IN FETCHING wheel handle for: %s", wheel_joint_names[i].c_str());
      return CallbackReturn::ERROR;
    }
    wheel_handles_[i]->set_velocity(0.0);
  }

  for (std::size_t i = 0; i < axle_handles_.size(); ++i)
  {
    if (!axle_handles_[i])
    {
      RCLCPP_ERROR(logger, "ERROR IN FETCHING axle handle for: %s", axle_joint_names[i].c_str());
      return CallbackReturn::ERROR;
    }
    axle_handles_[i]->set_position(0.0);
    previous_steering_angles_[i] = axle_handles_[i]->get_feedback();
  }

  is_halted_ = false;
  subscriber_is_active_ = true;
  RCLCPP_INFO(logger, "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();

  if (this->get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted_)
    {
      halt();
      is_halted_ = true;
    }

    return controller_interface::return_type::OK;
  }

  const auto current_time = time;

  std::shared_ptr<TwistStamped> last_command_msg = *(received_velocity_msg_ptr_.readFromRT());

  if (last_command_msg == nullptr)
  {
    last_command_msg = std::make_shared<TwistStamped>();
    last_command_msg->header.stamp = current_time;
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
    received_velocity_msg_ptr_.writeFromNonRT(last_command_msg);
  }

  const auto age_of_last_command = current_time - last_command_msg->header.stamp;

  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  TwistStamped command = *last_command_msg;
  double & linear_x_cmd = command.twist.linear.x;
  double & linear_y_cmd = command.twist.linear.y;
  double & angular_cmd = command.twist.angular.z;

  auto wheel_command = swerveDriveKinematics_.compute_wheel_commands(
    linear_x_cmd, linear_y_cmd, angular_cmd, params_.wheel_radius);

  std::vector<std::tuple<WheelCommand &, double, std::string>> wheel_data = {
    {wheel_command[0], params_.front_left_velocity_threshold / params_.wheel_radius,
     "front_left_wheel"},
    {wheel_command[1], params_.front_right_velocity_threshold / params_.wheel_radius,
     "front_right_wheel"},
    {wheel_command[2], params_.rear_left_velocity_threshold / params_.wheel_radius,
     "rear_left_wheel"},
    {wheel_command[3], params_.rear_right_velocity_threshold / params_.wheel_radius,
     "rear_right_wheel"}};

  for (const auto & [wheel_command_, threshold, label] : wheel_data)
  {
    if (wheel_command_.drive_velocity > threshold)
    {
      wheel_command_.drive_velocity = threshold;
    }
  }

  for (std::size_t i = 0; i < 4; i++)
  {
    if (!axle_handles_[i].has_value() || !wheel_handles_[i].has_value())
    {
      throw std::runtime_error(
        "Axle or Wheel handle is nullptr for: " + axle_joint_names[i] + " / " +
        wheel_joint_names[i]);
    }

    const bool is_stop = (std::fabs(linear_x_cmd) < EPS) && (std::fabs(linear_y_cmd) < EPS) &&
                         (std::fabs(angular_cmd) < EPS);

    if (is_stop)
    {
      axle_handles_[i]->set_position(previous_steering_angles_[i]);
    }
    else
    {
      axle_handles_[i]->set_position(wheel_command[i].steering_angle);
      previous_steering_angles_[i] = wheel_command[i].steering_angle;
    }
    wheel_handles_[i]->set_velocity(wheel_command[i].drive_angular_velocity);
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  swerve_drive_controller::OdometryState odometry_;

  std::array<double, 4> velocity_array{};
  std::array<double, 4> steering_angle_array{};

  for (std::size_t i = 0; i < 4; ++i)
  {
    if (params_.open_loop)
    {
      velocity_array[i] = wheel_command[i].drive_velocity;
      steering_angle_array[i] = wheel_command[i].steering_angle;
    }
    else
    {
      velocity_array[i] = wheel_handles_[i]->get_feedback() * params_.wheel_radius;
      steering_angle_array[i] = axle_handles_[i]->get_feedback();
    }
  }
  odometry_ = swerveDriveKinematics_.update_odometry(
    velocity_array, steering_angle_array, update_dt.seconds());

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.theta);

  if (realtime_odometry_publisher_)
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = time;
    odometry_message.pose.pose.position.x = odometry_.x;
    odometry_message.pose.pose.position.y = odometry_.y;
    odometry_message.pose.pose.orientation.x = orientation.x();
    odometry_message.pose.pose.orientation.y = orientation.y();
    odometry_message.pose.pose.orientation.z = orientation.z();
    odometry_message.pose.pose.orientation.w = orientation.w();
    realtime_odometry_publisher_->tryPublish(odometry_message);
  }

  if (realtime_odometry_transform_publisher_)
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
    realtime_odometry_transform_publisher_->tryPublish(odometry_transform_message_);
  }
  previous_publish_timestamp_ = time;
  return controller_interface::return_type::OK;
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.writeFromNonRT(std::make_shared<TwistStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool SwerveController::reset()
{
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  auto zero_twist = std::make_shared<TwistStamped>();
  zero_twist->header.stamp = get_node()->get_clock()->now();
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.writeFromNonRT(zero_twist);
  is_halted_ = false;
  return true;
}

CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void SwerveController::halt()
{
  for (std::size_t i = 0; i < wheel_handles_.size(); ++i)
  {
    wheel_handles_[i]->set_velocity(0.0);
  }
  for (std::size_t i = 0; i < axle_handles_.size(); ++i)
  {
    axle_handles_[i]->set_position(0.0);
  }
}

}  // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_drive_controller::SwerveController, controller_interface::ControllerInterface)
