// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg =
  ackermann_steering_controller::AckermannSteeringController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace ackermann_steering_controller
{
AckermannSteeringController::AckermannSteeringController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn AckermannSteeringController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ackermann_steering_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannSteeringController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  const double wheel_seperation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheel_radius = params_.wheel_radius_multiplier * params_.wheel_radius;
  const double wheelbase = params_.wheelbase_multiplier * params_.wheelbase;
  odometry_.set_wheel_params(wheel_seperation, wheel_radius, wheelbase);
  odometry_.set_velocity_rolling_window_size(params_.velocity_rolling_window_size);

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  if (params_.use_stamped_vel)
  {
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
      "~/reference", subscribers_qos,
      std::bind(&AckermannSteeringController::reference_callback, this, std::placeholders::_1));
  }
  else
  {
    ref_subscriber_unstamped_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/reference_unstamped", subscribers_qos,
      std::bind(
        &AckermannSteeringController::reference_callback_unstamped, this, std::placeholders::_1));
  }

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);

  try
  {
    // Odom state publisher
    odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgOdom>(
      "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }
  rt_odom_state_publisher_->unlock();

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgTf>(
      "~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ =
      std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
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

  try
  {
    // State publisher
    controller_s_publisher_ = get_node()->create_publisher<AckermanControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->msg_.odom.pose.pose.position.z = 0;
  controller_state_publisher_->msg_.odom.child_frame_id = params_.base_frame_id;
  controller_state_publisher_->unlock();
  auto & covariance_controller = controller_state_publisher_->msg_.odom.twist.covariance;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance_controller[diagonal_index] = params_.pose_covariance_diagonal[index];
    covariance_controller[diagonal_index] = params_.twist_covariance_diagonal[index];
  }
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AckermannSteeringController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }
  const auto age_of_last_command = get_node()->now() - msg->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
  }
}

void AckermannSteeringController::reference_callback_unstamped(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  auto twist_stamped = *(input_ref_.readFromNonRT());
  twist_stamped->header.stamp = get_node()->now();
  // if no timestamp provided use current time for command timestamp
  if (twist_stamped->header.stamp.sec == 0 && twist_stamped->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    twist_stamped->header.stamp = get_node()->now();
  }

  const auto age_of_last_command = get_node()->now() - twist_stamped->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    twist_stamped->twist = *msg;
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(twist_stamped->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
  }
}

controller_interface::InterfaceConfiguration
AckermannSteeringController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(NR_CMD_ITFS);

  for (int i = 0; i < params_.front_steer_names.size(); i++)
  {
    command_interfaces_config.names.push_back(
      params_.rear_wheel_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (int i = 0; i < params_.front_steer_names.size(); i++)
  {
    command_interfaces_config.names.push_back(
      params_.front_steer_names[i] + "/" + hardware_interface::HW_IF_POSITION);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
AckermannSteeringController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(NR_STATE_ITFS);
  const auto rear_wheel_feedback = params_.position_feedback ? hardware_interface::HW_IF_POSITION
                                                             : hardware_interface::HW_IF_VELOCITY;

  for (int i = 0; i < params_.front_steer_names.size(); i++)
  {
    state_interfaces_config.names.push_back(
      params_.rear_wheel_names[i] + "/" + rear_wheel_feedback);
  }

  for (int i = 0; i < params_.front_steer_names.size(); i++)
  {
    state_interfaces_config.names.push_back(
      params_.front_steer_names[i] + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
AckermannSteeringController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(NR_REF_ITFS, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(NR_REF_ITFS);

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("linear/") + hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[0]));

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("angular/") + hardware_interface::HW_IF_POSITION,
    &reference_interfaces_[1]));

  return reference_interfaces;
}

bool AckermannSteeringController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn AckermannSteeringController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannSteeringController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < NR_CMD_ITFS; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AckermannSteeringController::update_reference_from_subscribers()
{
  auto current_ref = *(input_ref_.readFromRT());
  const auto age_of_last_command = get_node()->now() - (current_ref)->header.stamp;

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  // send message only if there is no timeout
  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
  {
    if (!std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.angular.z))
    {
      reference_interfaces_[0] = current_ref->twist.linear.x;
      reference_interfaces_[1] = current_ref->twist.angular.z;
    }
  }
  else
  {
    reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
    reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AckermannSteeringController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double rear_wheel_value = state_interfaces_[0].get_value();
    const double steer_position = state_interfaces_[1].get_value() * params_.steer_pos_multiplier;

    if (!std::isnan(rear_wheel_value) && !std::isnan(steer_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(rear_wheel_value, steer_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(rear_wheel_value, steer_position, period.seconds());
      }
    }
  }

  // MOVE ROBOT

  // Limit velocities and accelerations:
  // TODO(destogl): add limiter for the velocities

  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]))
  {
    // store and set commands
    const double linear_command = reference_interfaces_[0];
    const double angular_command = reference_interfaces_[1];
    auto [alpha_write, Ws_write] = odometry_.twist_to_ackermann(linear_command, angular_command);
    // previous_publish_timestamp_ = time;

    // omega = linear_vel / radius
    command_interfaces_[0].set_value(Ws_write);
    command_interfaces_[1].set_value(alpha_write);
  }

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || is_in_chained_mode())
  {
    reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
    reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  }

  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.get_heading());

  // Populate odom message and publish
  if (rt_odom_state_publisher_->trylock())
  {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.get_x();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.get_y();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.get_linear();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.get_angular();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x =
      odometry_.get_x();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y =
      odometry_.get_y();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.odom.pose.pose.position.x = odometry_.get_x();
    controller_state_publisher_->msg_.odom.pose.pose.position.y = odometry_.get_y();
    controller_state_publisher_->msg_.odom.pose.pose.orientation = tf2::toMsg(orientation);
    controller_state_publisher_->msg_.odom.twist.twist.linear.x = odometry_.get_linear();
    controller_state_publisher_->msg_.odom.twist.twist.angular.z = odometry_.get_angular();
    if (params_.position_feedback)
    {
      controller_state_publisher_->msg_.rear_wheel_position = state_interfaces_[0].get_value();
    }
    else
    {
      controller_state_publisher_->msg_.rear_wheel_velocity = state_interfaces_[0].get_value();
    }
    controller_state_publisher_->msg_.steer_position =
      state_interfaces_[1].get_value() * params_.steer_pos_multiplier;
    controller_state_publisher_->msg_.linear_velocity_command = command_interfaces_[0].get_value();
    controller_state_publisher_->msg_.steering_angle_command = command_interfaces_[1].get_value();

    controller_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace ackermann_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ackermann_steering_controller::AckermannSteeringController,
  controller_interface::ChainableControllerInterface)
