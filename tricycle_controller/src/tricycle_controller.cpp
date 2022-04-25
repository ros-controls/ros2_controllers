#include "tricycle_controller/tricycle_controller.hpp"
#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
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

const char * TricycleController::feedback_type() const
{
  return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

CallbackReturn TricycleController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("wheel_front_joint_name", std::string());
    auto_declare<std::string>("steering_joint_name", std::string());

    auto_declare<double>("wheel_separation", wheel_params_.separation);
    auto_declare<double>("wheel_radius", wheel_params_.radius);
    auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
    auto_declare<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
    auto_declare<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("position_feedback", odom_params_.position_feedback);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
    auto_declare<double>("publish_rate", publish_rate_);
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
  command_interfaces_config.names.push_back(wheel_front_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);
  return command_interfaces_config;
}

InterfaceConfiguration TricycleController::state_interface_configuration() const
{
  InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.push_back(wheel_front_joint_name_ + "/" + feedback_type());
  state_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);
  return state_interfaces_config;
}

controller_interface::return_type TricycleController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get current states
  double current_speed = wheel_front_joint_[0].feedback_state.get().get_value();  // in m/s
  double current_steering_angle =
    steering_joint_[0].position_state.get().get_value();  // in radians

  // Get the last velocity command
  std::shared_ptr<Twist> velocity_command;
  received_velocity_msg_ptr_.get(velocity_command);

  if (velocity_command == nullptr)
  {
    return controller_interface::return_type::OK;
  }

  // Calculate the wheel velocity
  const auto twist = *velocity_command;
  double baselink_speed = twist.twist.linear.x;
  double steering_angle = twist.twist.angular.z;

  double delta_diff = normalize_angle(steering_angle - current_steering_angle);

  // front wheel has to drive faster in turns
  double front_wheel_scale_factor = 1 / std::max(0.01, cos(steering_angle));

  // slow down when difference between commanded steering angle and current steering angle is large
  double scale_speed_direction_diff_factor = std::max(0.01, cos(delta_diff));

  double steering_wheel_speed =
    (baselink_speed * front_wheel_scale_factor * scale_speed_direction_diff_factor);

  wheel_front_joint_[0].velocity_command.get().set_value(steering_wheel_speed);
  steering_joint_[0].position_command.get().set_value(steering_angle);

  return controller_interface::return_type::OK;
}

CallbackReturn TricycleController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update parameters
  wheel_front_joint_name_ = get_node()->get_parameter("wheel_front_joint_name").as_string();
  steering_joint_name_ = get_node()->get_parameter("steering_joint_name").as_string();
  if (wheel_front_joint_name_.empty())
  {
    RCLCPP_ERROR(logger, "'wheel_front_joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }
  if (steering_joint_name_.empty())
  {
    RCLCPP_ERROR(logger, "'steering_joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  wheel_params_.separation = get_node()->get_parameter("wheel_separation").as_double();
  wheel_params_.radius = get_node()->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    get_node()->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier =
    get_node()->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier =
    get_node()->get_parameter("right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  // odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  // odometry_.setVelocityRollingWindowSize(
  //   get_node()->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
  odom_params_.position_feedback = get_node()->get_parameter("position_feedback").as_bool();
  odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = get_node()->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      get_node()->get_parameter("linear.x.has_velocity_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_jerk_limits").as_bool(),
      get_node()->get_parameter("linear.x.min_velocity").as_double(),
      get_node()->get_parameter("linear.x.max_velocity").as_double(),
      get_node()->get_parameter("linear.x.min_acceleration").as_double(),
      get_node()->get_parameter("linear.x.max_acceleration").as_double(),
      get_node()->get_parameter("linear.x.min_jerk").as_double(),
      get_node()->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(
      get_node()->get_parameter("angular.z.has_velocity_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_jerk_limits").as_bool(),
      get_node()->get_parameter("angular.z.min_velocity").as_double(),
      get_node()->get_parameter("angular.z.max_velocity").as_double(),
      get_node()->get_parameter("angular.z.min_acceleration").as_double(),
      get_node()->get_parameter("angular.z.max_acceleration").as_double(),
      get_node()->get_parameter("angular.z.min_jerk").as_double(),
      get_node()->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
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
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = get_node()->get_clock()->now();

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

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "On activate: Initialize Joints");

  // Initialize the joints
  const auto wheel_front_result = get_wheel(wheel_front_joint_name_, wheel_front_joint_);
  const auto steering_result = get_steer(steering_joint_name_, steering_joint_);
  if (wheel_front_result == CallbackReturn::ERROR || steering_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }
  if (wheel_front_joint_.empty() || steering_joint_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either left wheel interfaces, right wheel interfaces are non existent");
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

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
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

bool TricycleController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  // TODO: clear handles
  // wheel_front_joint_.clear();
  // steering_joint_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn TricycleController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void TricycleController::halt() { wheel_front_joint_[0].velocity_command.get().set_value(0.0); }

CallbackReturn TricycleController::get_wheel(
  const std::string & wheel_joint_name, std::vector<TractionHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  // Lookup the velocity state interface
  const auto interface_name = feedback_type();
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&wheel_joint_name, &interface_name](const auto & interface)
    {
      return interface.get_name() == wheel_joint_name &&
             interface.get_interface_name() == interface_name;
    });
  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      wheel_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Lookup the velocity command interface
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
      return interface.get_name() == wheel_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      wheel_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the wheel joint instance
  joint.emplace_back(TractionHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

CallbackReturn TricycleController::get_steer(
  const std::string & steering_joint_name, std::vector<SteeringHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Steering Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_name() == steering_joint_name &&
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
    [&steering_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
      return interface.get_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  // Create the wheel joint instance
  joint.emplace_back(SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

double TricycleController::normalize_angle(double angle)
{
  while (angle > M_PI)
  {
    angle = angle - 2.0 * M_PI;
  }

  while (angle < -M_PI)
  {
    angle = angle + 2.0 * M_PI;
  }

  return angle;
}

double TricycleController::convert_trans_rot_vel_to_steering_angle(
  double v, double omega, double wheelbase)
{
  if (omega == 0 || v == 0)
  {
    return 0;
  }
  double radius = v / omega;
  return std::atan(wheelbase / radius);
}

}  // namespace tricycle_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tricycle_controller::TricycleController, controller_interface::ControllerInterface)
