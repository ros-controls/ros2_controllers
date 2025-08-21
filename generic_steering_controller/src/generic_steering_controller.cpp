#include <memory>
#include "generic_steering_controller/generic_steering_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace
{  // utility

using ControllerTwistReferenceMsg =
  generic_steering_controller::GenericSteeringController::ControllerTwistReferenceMsg;

void reset_controller_reference_msg(
  ControllerTwistReferenceMsg & msg, const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg.header.stamp = node->now();
  msg.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace generic_steering_controller
{
GenericSteeringController::GenericSteeringController()
: controller_interface::ChainableControllerInterface(),
  kinematic_loader("generic_steering_controller", "kinematic_model::KinematicModelBase")
{
}

controller_interface::CallbackReturn GenericSteeringController::on_init()
{
  try {
    param_listener_ =
      std::make_shared<generic_steering_controller_parameters::ParamListener>(get_node());
    initialize_implementation_parameter_listener();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GenericSteeringController::set_interface_numbers(
  size_t nr_state_itfs = 2, size_t nr_cmd_itfs = 2, size_t nr_ref_itfs = 2)
{
  nr_state_itfs_ = nr_state_itfs;
  nr_cmd_itfs_ = nr_cmd_itfs;
  nr_ref_itfs_ = nr_ref_itfs;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GenericSteeringController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.traction_joints_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No traction joints specified!");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.steering_joints_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No steering joints specified!");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Params created");

  traction_joints_state_names_ = params_.traction_joints_names;
  steering_joints_state_names_ = params_.steering_joints_names;

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_timeout_ = rclcpp::Duration::from_seconds(params_.ref_timeout);
  ref_subscriber_twist_ = get_node()->create_subscription<ControllerTwistReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&GenericSteeringController::reference_callback, this, std::placeholders::_1));

  reset_controller_reference_msg(current_ref_, get_node());
  input_ref_.set(current_ref_);

  try {
    odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgOdom>(
    "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;
  rt_odom_state_publisher_->unlock();

  auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }


  try {
    // Tf State publisher
    tf_odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgTf>(
      "~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ =
      std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  try {
    // State publisher
    controller_s_publisher_ = get_node()->create_publisher<GenericSteeringControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->unlock();
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  size_t total_state_interfaces = traction_joints_state_names_.size() +
    steering_joints_state_names_.size();
  size_t total_command_interfaces = params_.traction_joints_names.size() +
    params_.steering_joints_names.size();

  set_interface_numbers(total_state_interfaces, total_command_interfaces, 2);
  open_loop_ = params_.open_loop;
  try {
    kinematic_model_ = kinematic_loader.createSharedInstance(params_.plugin_name);
    kinematic_model_->configure(
    std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(get_node()),
    params_.traction_joints_names,
    params_.steering_joints_names
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load kinematic plugin: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;

}

void GenericSteeringController::reference_callback(
  const std::shared_ptr<ControllerTwistReferenceMsg> msg)
{
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }
  const auto age_of_last_command = get_node()->now() - msg->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_) {
    input_ref_.set(*msg);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
  }
}

controller_interface::InterfaceConfiguration
GenericSteeringController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);
  for (size_t i = 0; i < params_.traction_joints_names.size(); i++) {
    command_interfaces_config.names.push_back(
      params_.traction_joints_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (size_t i = 0; i < params_.steering_joints_names.size(); i++) {
    command_interfaces_config.names.push_back(
      params_.steering_joints_names[i] + "/" + hardware_interface::HW_IF_POSITION);
  }
  return command_interfaces_config;

}

controller_interface::InterfaceConfiguration
GenericSteeringController::state_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "State interface conf");
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(nr_state_itfs_);
  const auto traction_wheels_feedback = params_.position_feedback ?
    hardware_interface::HW_IF_POSITION :
    hardware_interface::HW_IF_VELOCITY;

  for (size_t i = 0; i < traction_joints_state_names_.size(); i++) {
    state_interfaces_config.names.push_back(
      traction_joints_state_names_[i] + "/" + traction_wheels_feedback);
  }

  // Add steering joint interfaces
  for (size_t i = 0; i < steering_joints_state_names_.size(); ++i) {
    state_interfaces_config.names.push_back(
      steering_joints_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
GenericSteeringController::on_export_reference_interfaces()
{
  RCLCPP_INFO(get_node()->get_logger(), "ref int exp conf: %zu", nr_ref_itfs_);
  reference_interfaces_.resize(nr_ref_itfs_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(nr_ref_itfs_);

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

bool GenericSteeringController::on_set_chained_mode(bool /*chained_mode*/) {return true;}

controller_interface::CallbackReturn GenericSteeringController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset reference message to NaN
  reset_controller_reference_msg(current_ref_, get_node());
  input_ref_.try_set(current_ref_);

  // Set all command interfaces to NaN for safety
  for (auto & command_interface : command_interfaces_) {
    if (!command_interface.set_value(std::numeric_limits<double>::quiet_NaN())) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Failed to set NaN value for command interface '%s' during activation.",
        command_interface.get_name().c_str());
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "GenericSteeringController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GenericSteeringController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all command interfaces to NaN for safety
  for (auto & command_interface : command_interfaces_) {
    if (!command_interface.set_value(std::numeric_limits<double>::quiet_NaN())) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Failed to set NaN value for command interface '%s' during deactivation.",
        command_interface.get_name().c_str());
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "GenericSteeringController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GenericSteeringController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Get the latest reference message from the subscriber
  auto current_ref_msg = input_ref_.try_get();
  if (current_ref_msg.has_value()) {
    current_ref_ = current_ref_msg.value();
  }
  const auto age_of_last_command = time - current_ref_msg.value().header.stamp;

  // Check if the reference is fresh and valid
  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
    if (!std::isnan(current_ref_.twist.linear.x) &&
        !std::isnan(current_ref_.twist.angular.z)
        /* && !std::isnan(current_ref_.twist.linear.y) */) // Uncomment if needed
    {
      reference_interfaces_[0] = current_ref_.twist.linear.x;
      reference_interfaces_[1] = current_ref_.twist.angular.z;

      RCLCPP_DEBUG(get_node()->get_logger(),
        "Accepted reference: linear.x=%.3f angular.z=%.3f age=%.3f",
        current_ref_.twist.linear.x, current_ref_.twist.angular.z, age_of_last_command.seconds());

      if (ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
        current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
        input_ref_.try_set(current_ref_);
        RCLCPP_DEBUG(get_node()->get_logger(), "Cleared reference after use (timeout==0)");
      }
    }
  } else {
    if (!std::isnan(current_ref_.twist.linear.x) && !std::isnan(current_ref_.twist.angular.z)) {
      reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
      reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();

      current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
      current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
      input_ref_.try_set(current_ref_);

      RCLCPP_WARN(get_node()->get_logger(),
        "Reference timed out (age=%.3f > timeout=%.3f), clearing reference.",
        age_of_last_command.seconds(), ref_timeout_.seconds());
    }
  }
  return controller_interface::return_type::OK;
}

// Call values from plugin and pass them to hardware interfaces
controller_interface::return_type GenericSteeringController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Check kinematic plugin
  if (!kinematic_model_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Kinematic model plugin is not loaded!");
    return controller_interface::return_type::ERROR;
  }

  // Update kinematic model states if not open loop
  if (!open_loop_) {
    state_map = get_state_interface_map();
    kinematic_model_->update_states(state_map);
  }

  // If reference is valid, compute and send commands
  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1])) {
    auto [traction_commands, steering_commands] = kinematic_model_->get_commands(
      reference_interfaces_[0], reference_interfaces_[1], params_.open_loop,
      params_.reduce_wheel_speed_until_steering_reached);

    // Write traction commands
    for (size_t i = 0; i < params_.traction_joints_names.size(); i++) {
      if (!command_interfaces_[i].set_value(traction_commands[i])) {
        RCLCPP_WARN(get_node()->get_logger(),
          "Unable to set traction command at index %zu: value = %f", i, traction_commands[i]);
      }
    }
    // Write steering commands
    for (size_t i = 0; i < params_.steering_joints_names.size(); i++) {
      size_t idx = i + params_.traction_joints_names.size();
      if (!command_interfaces_[idx].set_value(steering_commands[i])) {
        RCLCPP_WARN(get_node()->get_logger(),
          "Unable to set steering command at index %zu: value = %f", i, steering_commands[i]);
      }
    }
  } else {
    // Reference is invalid: zero traction commands
    for (size_t i = 0; i < params_.traction_joints_names.size(); i++) {
      if (!command_interfaces_[i].set_value(0.0)) {
        RCLCPP_WARN(get_node()->get_logger(),
          "Unable to set traction command to zero at index %zu", i);
      }
    }
    // Optionally zero steering commands (uncomment if desired)
    // for (size_t i = 0; i < params_.steering_joints_names.size(); i++) {
    //   size_t idx = i + params_.traction_joints_names.size();
    //   if (!command_interfaces_[idx].set_value(0.0)) {
    //     RCLCPP_WARN(get_node()->get_logger(),
    //       "Unable to set steering command to zero at index %zu", i);
    //   }
    // }
  }

  // Publish controller state message
  if (controller_state_publisher_->trylock()) {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.traction_wheels_position.clear();
    controller_state_publisher_->msg_.traction_wheels_velocity.clear();
    controller_state_publisher_->msg_.traction_command.clear();
    controller_state_publisher_->msg_.steer_positions.clear();
    controller_state_publisher_->msg_.steering_angle_command.clear();

    auto number_of_traction_wheels = params_.traction_joints_names.size();
    auto number_of_steering_wheels = params_.steering_joints_names.size();

    // Read state interfaces using get_optional and log missing values
    for (size_t i = 0; i < number_of_traction_wheels; ++i) {
      auto value_op = state_interfaces_[i].get_optional();
      if (params_.position_feedback) {
        controller_state_publisher_->msg_.traction_wheels_position.push_back(value_op.value_or(0.0));
      } else {
        controller_state_publisher_->msg_.traction_wheels_velocity.push_back(value_op.value_or(0.0));
      }
      if (!value_op.has_value()) {
        RCLCPP_DEBUG(get_node()->get_logger(),
          "Unable to retrieve %s feedback data for traction wheel %zu",
          params_.position_feedback ? "position" : "velocity", i);
      }
    }

    for (size_t i = 0; i < number_of_steering_wheels; ++i) {
      size_t steering_base_index = number_of_traction_wheels;
      auto steer_state_op = state_interfaces_[steering_base_index + i].get_optional();
      controller_state_publisher_->msg_.steer_positions.push_back(steer_state_op.value_or(0.0));
      if (!steer_state_op.has_value()) {
        RCLCPP_DEBUG(get_node()->get_logger(),
          "Unable to retrieve position feedback data for steering wheel %zu", i);
      }

      auto steer_cmd_op = command_interfaces_[number_of_traction_wheels + i].get_optional();
      controller_state_publisher_->msg_.steering_angle_command.push_back(steer_cmd_op.value_or(0.0));
      if (!steer_cmd_op.has_value()) {
        RCLCPP_DEBUG(get_node()->get_logger(),
          "Unable to retrieve command interface value for steering wheel %zu", i);
      }
    }

    controller_state_publisher_->unlockAndPublish();
  }

  // Publish odometry and tf messages (unchanged)
  auto odom_msg = kinematic_model_->get_odometry_message(period);
  const auto & q = odom_msg->pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);

  if (rt_odom_state_publisher_->trylock()) {
    rt_odom_state_publisher_->msg_.header = odom_msg->header;
    rt_odom_state_publisher_->msg_.pose = odom_msg->pose;
    rt_odom_state_publisher_->unlockAndPublish();
  }

  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock()) {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x =
      odom_msg->pose.pose.position.x;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y =
      odom_msg->pose.pose.position.y;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      odom_msg->pose.pose.orientation;
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  // Clear reference after use
  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

void GenericSteeringController::initialize_implementation_parameter_listener()
{

  RCLCPP_DEBUG(get_node()->get_logger(), "Generic steering controller parameters initialized");
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  generic_steering_controller::GenericSteeringController,
  controller_interface::ChainableControllerInterface)
