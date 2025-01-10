// Copyright 2022 Pixel Robotics.
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
 * Author: Tony Najjar
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tricycle_controller/tricycle_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ACKERMANN_OUT_TOPIC = "~/cmd_ackermann";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_RESET_ODOM_SERVICE = "~/reset_odometry";
}  // namespace

namespace tricycle_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

TricycleController::TricycleController() : controller_interface::ControllerInterface() {}

CallbackReturn TricycleController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("traction_joint_name", std::string());
    auto_declare<std::string>("steering_joint_name", std::string());

    auto_declare<double>("wheelbase", wheel_params_.wheelbase);
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);
    auto_declare<bool>("odom_only_twist", odom_params_.odom_only_twist);

    auto_declare<int>("cmd_vel_timeout", static_cast<int>(cmd_vel_timeout_.count()));
    auto_declare<bool>("publish_ackermann_command", publish_ackermann_command_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<double>("traction.max_velocity", NAN);
    auto_declare<double>("traction.min_velocity", NAN);
    auto_declare<double>("traction.max_acceleration", NAN);
    auto_declare<double>("traction.min_acceleration", NAN);
    auto_declare<double>("traction.max_deceleration", NAN);
    auto_declare<double>("traction.min_deceleration", NAN);
    auto_declare<double>("traction.max_jerk", NAN);
    auto_declare<double>("traction.min_jerk", NAN);

    auto_declare<double>("steering.max_position", NAN);
    auto_declare<double>("steering.min_position", NAN);
    auto_declare<double>("steering.max_velocity", NAN);
    auto_declare<double>("steering.min_velocity", NAN);
    auto_declare<double>("steering.max_acceleration", NAN);
    auto_declare<double>("steering.min_acceleration", NAN);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration TricycleController::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(traction_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);
  return command_interfaces_config;
}

InterfaceConfiguration TricycleController::state_interface_configuration() const
{
  InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.push_back(traction_joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);
  return state_interfaces_config;
}

controller_interface::return_type TricycleController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }
  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);
  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by Limiters,
  // without affecting the stored twist command
  TwistStamped command = *last_command_msg;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;
  double Ws_read = traction_joint_[0].velocity_state.get().get_value();     // in radians/s
  double alpha_read = steering_joint_[0].position_state.get().get_value();  // in radians

  if (odom_params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, period);
  }
  else
  {
    if (std::isnan(Ws_read) || std::isnan(alpha_read))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not read feedback value");
      return controller_interface::return_type::ERROR;
    }
    odometry_.update(Ws_read, alpha_read, period);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (realtime_odometry_publisher_->trylock())
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = time;
    if (!odom_params_.odom_only_twist)
    {
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
    }
    odometry_message.twist.twist.linear.x = odometry_.getLinear();
    odometry_message.twist.twist.angular.z = odometry_.getAngular();
    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }

  // Compute wheel velocity and angle
  auto [alpha_write, Ws_write] = twist_to_ackermann(linear_command, angular_command);

  // Reduce wheel speed until the target angle has been reached
  double alpha_delta = abs(alpha_write - alpha_read);
  double scale;
  if (alpha_delta < M_PI / 6)
  {
    scale = 1;
  }
  else if (alpha_delta > M_PI_2)
  {
    scale = 0.01;
  }
  else
  {
    // TODO(anyone): find the best function, e.g convex power functions
    scale = cos(alpha_delta);
  }
  Ws_write *= scale;

  auto & last_command = previous_commands_.back();
  auto & second_to_last_command = previous_commands_.front();

  limiter_traction_.limit(
    Ws_write, last_command.speed, second_to_last_command.speed, period.seconds());

  limiter_steering_.limit(
    alpha_write, last_command.steering_angle, second_to_last_command.steering_angle,
    period.seconds());

  previous_commands_.pop();
  AckermannDrive ackermann_command;
  // speed in AckermannDrive is defined as desired forward speed (m/s) but it is used here as wheel
  // speed (rad/s)
  ackermann_command.speed = static_cast<float>(Ws_write);
  ackermann_command.steering_angle = static_cast<float>(alpha_write);
  previous_commands_.emplace(ackermann_command);

  //  Publish ackermann command
  if (publish_ackermann_command_ && realtime_ackermann_command_publisher_->trylock())
  {
    auto & realtime_ackermann_command = realtime_ackermann_command_publisher_->msg_;
    // speed in AckermannDrive is defined desired forward speed (m/s) but we use it here as wheel
    // speed (rad/s)
    realtime_ackermann_command.speed = static_cast<float>(Ws_write);
    realtime_ackermann_command.steering_angle = static_cast<float>(alpha_write);
    realtime_ackermann_command_publisher_->unlockAndPublish();
  }

  traction_joint_[0].velocity_command.get().set_value(Ws_write);
  steering_joint_[0].position_command.get().set_value(alpha_write);
  return controller_interface::return_type::OK;
}

CallbackReturn TricycleController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update parameters
  traction_joint_name_ = get_node()->get_parameter("traction_joint_name").as_string();
  steering_joint_name_ = get_node()->get_parameter("steering_joint_name").as_string();
  if (traction_joint_name_.empty())
  {
    RCLCPP_ERROR(logger, "'traction_joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }
  if (steering_joint_name_.empty())
  {
    RCLCPP_ERROR(logger, "'steering_joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  wheel_params_.wheelbase = get_node()->get_parameter("wheelbase").as_double();
  wheel_params_.radius = get_node()->get_parameter("wheel_radius").as_double();

  odometry_.setWheelParams(wheel_params_.wheelbase, wheel_params_.radius);
  odometry_.setVelocityRollingWindowSize(
    static_cast<size_t>(get_node()->get_parameter("velocity_rolling_window_size").as_int()));

  odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();
  odom_params_.odom_only_twist = get_node()->get_parameter("odom_only_twist").as_bool();

  cmd_vel_timeout_ =
    std::chrono::milliseconds{get_node()->get_parameter("cmd_vel_timeout").as_int()};
  publish_ackermann_command_ = get_node()->get_parameter("publish_ackermann_command").as_bool();
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_traction_ = TractionLimiter(
      get_node()->get_parameter("traction.min_velocity").as_double(),
      get_node()->get_parameter("traction.max_velocity").as_double(),
      get_node()->get_parameter("traction.min_acceleration").as_double(),
      get_node()->get_parameter("traction.max_acceleration").as_double(),
      get_node()->get_parameter("traction.min_deceleration").as_double(),
      get_node()->get_parameter("traction.max_deceleration").as_double(),
      get_node()->get_parameter("traction.min_jerk").as_double(),
      get_node()->get_parameter("traction.max_jerk").as_double());
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring traction limiter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try
  {
    limiter_steering_ = SteeringLimiter(
      get_node()->get_parameter("steering.min_position").as_double(),
      get_node()->get_parameter("steering.max_position").as_double(),
      get_node()->get_parameter("steering.min_velocity").as_double(),
      get_node()->get_parameter("steering.max_velocity").as_double(),
      get_node()->get_parameter("steering.min_acceleration").as_double(),
      get_node()->get_parameter("steering.max_acceleration").as_double());
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring steering limiter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  const TwistStamped empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>(empty_twist));

  // Fill last two commands with default constructed commands
  const AckermannDrive empty_ackermann_drive;
  previous_commands_.emplace(empty_ackermann_drive);
  previous_commands_.emplace(empty_ackermann_drive);

  // initialize ackermann command publisher
  if (publish_ackermann_command_)
  {
    ackermann_command_publisher_ = get_node()->create_publisher<AckermannDrive>(
      DEFAULT_ACKERMANN_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_ackermann_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AckermannDrive>>(
        ackermann_command_publisher_);
  }

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<TwistStamped> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<TwistStamped> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = get_node()->get_clock()->now();
      });
  }

  // initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  if (odom_params_.enable_odom_tf)
  {
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);

    // keeping track of odom and base_link transforms only
    auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;
  }

  // Create odom reset service
  reset_odom_service_ = get_node()->create_service<std_srvs::srv::Empty>(
    DEFAULT_RESET_ODOM_SERVICE, std::bind(
                                  &TricycleController::reset_odometry, this, std::placeholders::_1,
                                  std::placeholders::_2, std::placeholders::_3));

  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "On activate: Initialize Joints");

  // Initialize the joints
  const auto wheel_front_result = get_traction(traction_joint_name_, traction_joint_);
  const auto steering_result = get_steering(steering_joint_name_, steering_joint_);
  if (wheel_front_result == CallbackReturn::ERROR || steering_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }
  if (traction_joint_.empty() || steering_joint_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Either steering or traction interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void TricycleController::reset_odometry(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  odometry_.resetOdometry();
  RCLCPP_INFO(get_node()->get_logger(), "Odometry successfully reset");
}

bool TricycleController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<AckermannDrive> empty_ackermann_drive;
  std::swap(previous_commands_, empty_ackermann_drive);

  traction_joint_.clear();
  steering_joint_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

void TricycleController::halt()
{
  traction_joint_[0].velocity_command.get().set_value(0.0);
  steering_joint_[0].position_command.get().set_value(0.0);
}

CallbackReturn TricycleController::get_traction(
  const std::string & traction_joint_name, std::vector<TractionHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the traction joint instance
  joint.emplace_back(TractionHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::get_steering(
  const std::string & steering_joint_name, std::vector<SteeringHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Steering Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the steering joint instance
  joint.emplace_back(SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

double TricycleController::convert_trans_rot_vel_to_steering_angle(
  double Vx, double theta_dot, double wheelbase)
{
  if (theta_dot == 0 || Vx == 0)
  {
    return 0;
  }
  return std::atan(theta_dot * wheelbase / Vx);
}

std::tuple<double, double> TricycleController::twist_to_ackermann(double Vx, double theta_dot)
{
  // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  double alpha, Ws;

  if (Vx == 0 && theta_dot != 0)
  {  // is spin action
    alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
    Ws = abs(theta_dot) * wheel_params_.wheelbase / wheel_params_.radius;
    return std::make_tuple(alpha, Ws);
  }

  alpha = convert_trans_rot_vel_to_steering_angle(Vx, theta_dot, wheel_params_.wheelbase);
  Ws = Vx / (wheel_params_.radius * std::cos(alpha));
  return std::make_tuple(alpha, Ws);
}

}  // namespace tricycle_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tricycle_controller::TricycleController, controller_interface::ControllerInterface)
