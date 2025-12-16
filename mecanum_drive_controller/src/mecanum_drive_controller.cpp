// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "controller_interface/tf_prefix.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

using ControllerReferenceMsg =
  mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  ControllerReferenceMsg & msg, const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
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

namespace mecanum_drive_controller
{
MecanumDriveController::MecanumDriveController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<mecanum_drive_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  auto prepare_lists_with_joint_names =
    [&command_joints = this->command_joint_names_, &state_joints = this->state_joint_names_](
      const std::size_t index, const std::string & command_joint_name,
      const std::string & state_joint_name)
  {
    command_joints[index] = command_joint_name;
    if (state_joint_name.empty())
    {
      state_joints[index] = command_joint_name;
    }
    else
    {
      state_joints[index] = state_joint_name;
    }
  };

  command_joint_names_.resize(4);
  state_joint_names_.resize(4);

  // The joint names are sorted according to the order documented in the header file!
  prepare_lists_with_joint_names(
    FRONT_LEFT, params_.front_left_wheel_command_joint_name,
    params_.front_left_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    FRONT_RIGHT, params_.front_right_wheel_command_joint_name,
    params_.front_right_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    REAR_RIGHT, params_.rear_right_wheel_command_joint_name,
    params_.rear_right_wheel_state_joint_name);
  prepare_lists_with_joint_names(
    REAR_LEFT, params_.rear_left_wheel_command_joint_name,
    params_.rear_left_wheel_state_joint_name);

  // Initialize odometry
  std::array<double, PLANAR_POINT_DIM> base_frame_offset = {
    {params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y,
     params_.kinematics.base_frame_offset.theta}};
  odometry_.init(get_node()->now(), base_frame_offset);

  // Set wheel params for the odometry computation
  odometry_.setWheelsParams(
    params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis,
    params_.kinematics.wheels_radius);

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&MecanumDriveController::reference_callback, this, std::placeholders::_1));

  reset_controller_reference_msg(current_ref_, get_node());
  input_ref_.set(current_ref_);

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

  try
  {
    // Odom state publisher
    odom_s_publisher_ =
      get_node()->create_publisher<OdomStateMsg>("~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // resolve prefix: substitute tilde (~) with the namespace if contains and normalize slashes (/)
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = controller_interface::resolve_tf_prefix(
        params_.tf_frame_prefix, get_node()->get_namespace());
    }
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Please use tilde ('~') character in 'tf_frame_prefix' as it replaced with node namespace");

      tf_prefix = std::string(get_node()->get_namespace());
      if (tf_prefix.back() != '/')
      {
        tf_prefix = tf_prefix + "/";
      }
      if (tf_prefix.front() == '/')
      {
        tf_prefix.erase(0, 1);
      }
    }
  }

  // prepend resolved TF prefix to frame ids
  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  odom_state_msg_.header.stamp = get_node()->now();
  odom_state_msg_.header.frame_id = odom_frame_id;
  odom_state_msg_.child_frame_id = base_frame_id;
  odom_state_msg_.pose.pose.position.z = 0;

  auto & pose_covariance = odom_state_msg_.pose.covariance;
  auto & twist_covariance = odom_state_msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    pose_covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    twist_covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<TfStateMsg>("~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  tf_odom_state_msg_.transforms.resize(1);
  tf_odom_state_msg_.transforms[0].header.stamp = get_node()->now();
  tf_odom_state_msg_.transforms[0].header.frame_id = odom_frame_id;
  tf_odom_state_msg_.transforms[0].child_frame_id = base_frame_id;
  tf_odom_state_msg_.transforms[0].transform.translation.z = 0.0;

  try
  {
    // controller State publisher
    controller_s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr,
      "Exception thrown during publisher creation at configure stage "
      "with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_msg_.header.stamp = get_node()->now();
  controller_state_msg_.header.frame_id = odom_frame_id;

  RCLCPP_INFO(get_node()->get_logger(), "MecanumDriveController configured successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumDriveController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // If no timestamp provided, use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }

  const auto age_of_last_command = get_node()->now() - msg->header.stamp;

  // Check the timeout condition
  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.set(*msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older by %.10f than allowed timeout (%.4f).",
      rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());

    ControllerReferenceMsg emtpy_msg;
    reset_controller_reference_msg(emtpy_msg, get_node());
    input_ref_.set(emtpy_msg);
  }
}

controller_interface::InterfaceConfiguration
MecanumDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(command_joint_names_.size());
  for (const auto & joint : command_joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joint_names_.size());

  for (const auto & joint : state_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
MecanumDriveController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(NR_REF_ITFS, std::numeric_limits<double>::quiet_NaN());

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

bool MecanumDriveController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::CallbackReturn MecanumDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Try to set default value in command.
  // If this fails, then another command will be received soon anyways.
  ControllerReferenceMsg emtpy_msg;
  reset_controller_reference_msg(emtpy_msg, get_node());
  input_ref_.try_set(emtpy_msg);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bool value_set_no_error = true;
  for (size_t i = 0; i < NR_CMD_ITFS; ++i)
  {
    value_set_no_error &=
      command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  if (!value_set_no_error)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Setting values to command interfaces has failed! "
      "This means that you are maybe blocking the interface in your hardware for too long.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumDriveController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref_op = input_ref_.try_get();
  if (current_ref_op.has_value())
  {
    current_ref_ = current_ref_op.value();
  }

  const auto age_of_last_command = time - current_ref_.header.stamp;

  // accept message only if there is no timeout
  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
  {
    if (
      !std::isnan(current_ref_.twist.linear.x) && !std::isnan(current_ref_.twist.linear.y) &&
      !std::isnan(current_ref_.twist.angular.z))
    {
      reference_interfaces_[0] = current_ref_.twist.linear.x;
      reference_interfaces_[1] = current_ref_.twist.linear.y;
      reference_interfaces_[2] = current_ref_.twist.angular.z;

      if (ref_timeout_ == rclcpp::Duration::from_seconds(0))
      {
        current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        current_ref_.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

        input_ref_.try_set(current_ref_);
      }
    }
  }
  else
  {
    if (
      !std::isnan(current_ref_.twist.linear.x) && !std::isnan(current_ref_.twist.linear.y) &&
      !std::isnan(current_ref_.twist.angular.z))
    {
      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      reference_interfaces_[2] = 0.0;

      current_ref_.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
      current_ref_.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
      current_ref_.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

      input_ref_.try_set(current_ref_);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type MecanumDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // FORWARD KINEMATICS (odometry).
  const auto wheel_front_left_state_vel_op = state_interfaces_[FRONT_LEFT].get_optional();
  const auto wheel_front_right_state_vel_op = state_interfaces_[FRONT_RIGHT].get_optional();
  const auto wheel_rear_right_state_vel_op = state_interfaces_[REAR_RIGHT].get_optional();
  const auto wheel_rear_left_state_vel_op = state_interfaces_[REAR_LEFT].get_optional();

  if (
    !wheel_front_left_state_vel_op.has_value() || !wheel_front_right_state_vel_op.has_value() ||
    !wheel_rear_right_state_vel_op.has_value() || !wheel_rear_left_state_vel_op.has_value())
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Unable to retrieve data from front left wheel or front right wheel or rear left wheel or "
      "rear right wheel");
    return controller_interface::return_type::OK;
  }

  const double wheel_front_left_state_vel = wheel_front_left_state_vel_op.value();
  const double wheel_front_right_state_vel = wheel_front_right_state_vel_op.value();
  const double wheel_rear_right_state_vel = wheel_rear_right_state_vel_op.value();
  const double wheel_rear_left_state_vel = wheel_rear_left_state_vel_op.value();

  if (
    !std::isnan(wheel_front_left_state_vel) && !std::isnan(wheel_rear_left_state_vel) &&
    !std::isnan(wheel_rear_right_state_vel) && !std::isnan(wheel_front_right_state_vel))
  {
    // Estimate twist (using joint information) and integrate
    odometry_.update(
      wheel_front_left_state_vel, wheel_rear_left_state_vel, wheel_rear_right_state_vel,
      wheel_front_right_state_vel, period.seconds());
  }

  // INVERSE KINEMATICS (move robot).
  // Compute wheels velocities (this is the actual ik):
  // NOTE: the input desired twist (from topic `~/reference`) is a body twist.
  if (
    !std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) &&
    !std::isnan(reference_interfaces_[2]))
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, params_.kinematics.base_frame_offset.theta);
    /// \note The variables meaning:
    /// rotation_from_base_to_center: Rotation transformation matrix, to transform from
    /// base frame to center frame
    /// linear_trans_from_base_to_center: offset/linear transformation matrix, to
    /// transform from base frame to center frame

    tf2::Matrix3x3 rotation_from_base_to_center = tf2::Matrix3x3((quaternion));
    tf2::Vector3 velocity_in_base_frame_w_r_t_center_frame_ =
      rotation_from_base_to_center *
      tf2::Vector3(reference_interfaces_[0], reference_interfaces_[1], 0.0);
    tf2::Vector3 linear_trans_from_base_to_center = tf2::Vector3(
      params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y, 0.0);

    velocity_in_center_frame_linear_x_ =
      velocity_in_base_frame_w_r_t_center_frame_.x() +
      linear_trans_from_base_to_center.y() * reference_interfaces_[2];
    velocity_in_center_frame_linear_y_ =
      velocity_in_base_frame_w_r_t_center_frame_.y() -
      linear_trans_from_base_to_center.x() * reference_interfaces_[2];
    velocity_in_center_frame_angular_z_ = reference_interfaces_[2];

    const double wheel_front_left_vel =
      1.0 / params_.kinematics.wheels_radius *
      (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ -
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_);
    const double wheel_front_right_vel =
      1.0 / params_.kinematics.wheels_radius *
      (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ +
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_);
    const double wheel_rear_right_vel =
      1.0 / params_.kinematics.wheels_radius *
      (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ +
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_);
    const double wheel_rear_left_vel =
      1.0 / params_.kinematics.wheels_radius *
      (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
       params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
         velocity_in_center_frame_angular_z_);

    // Set wheels velocities - The joint names are sorted according to the order documented in the
    // header file!
    const bool value_set_error =
      command_interfaces_[FRONT_LEFT].set_value(wheel_front_left_vel) &&
      command_interfaces_[FRONT_RIGHT].set_value(wheel_front_right_vel) &&
      command_interfaces_[REAR_RIGHT].set_value(wheel_rear_right_vel) &&
      command_interfaces_[REAR_LEFT].set_value(wheel_rear_left_vel);
    RCLCPP_ERROR_EXPRESSION(
      get_node()->get_logger(), !value_set_error,
      "Setting values to command interfaces has failed! "
      "This means that you are maybe blocking the interface in your hardware for too long.");
  }
  else
  {
    const bool value_set_error = command_interfaces_[FRONT_LEFT].set_value(0.0) &&
                                 command_interfaces_[FRONT_RIGHT].set_value(0.0) &&
                                 command_interfaces_[REAR_RIGHT].set_value(0.0) &&
                                 command_interfaces_[REAR_LEFT].set_value(0.0);
    RCLCPP_ERROR_EXPRESSION(
      get_node()->get_logger(), !value_set_error,
      "Setting values to command interfaces has failed! "
      "This means that you are maybe blocking the interface in your hardware for too long.");
  }

  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getRz());

  // Populate odom message and publish
  if (rt_odom_state_publisher_)
  {
    odom_state_msg_.header.stamp = time;
    odom_state_msg_.pose.pose.position.x = odometry_.getX();
    odom_state_msg_.pose.pose.position.y = odometry_.getY();
    odom_state_msg_.pose.pose.orientation = tf2::toMsg(orientation);
    odom_state_msg_.twist.twist.linear.x = odometry_.getVx();
    odom_state_msg_.twist.twist.linear.y = odometry_.getVy();
    odom_state_msg_.twist.twist.angular.z = odometry_.getWz();
    rt_odom_state_publisher_->try_publish(odom_state_msg_);
  }

  // Publish tf /odom frame
  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_)
  {
    tf_odom_state_msg_.transforms.front().header.stamp = time;
    tf_odom_state_msg_.transforms.front().transform.translation.x = odometry_.getX();
    tf_odom_state_msg_.transforms.front().transform.translation.y = odometry_.getY();
    tf_odom_state_msg_.transforms.front().transform.rotation = tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->try_publish(tf_odom_state_msg_);
  }

  if (controller_state_publisher_)
  {
    controller_state_msg_.header.stamp = get_node()->now();

    controller_state_msg_.front_left_wheel_velocity = wheel_front_left_state_vel;
    controller_state_msg_.front_right_wheel_velocity = wheel_front_right_state_vel;
    controller_state_msg_.back_right_wheel_velocity = wheel_rear_right_state_vel;
    controller_state_msg_.back_left_wheel_velocity = wheel_rear_left_state_vel;

    controller_state_msg_.reference_velocity.linear.x = reference_interfaces_[0];
    controller_state_msg_.reference_velocity.linear.y = reference_interfaces_[1];
    controller_state_msg_.reference_velocity.angular.z = reference_interfaces_[2];
    controller_state_publisher_->try_publish(controller_state_msg_);
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

}  // namespace mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecanum_drive_controller::MecanumDriveController,
  controller_interface::ChainableControllerInterface)
