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

#include "steering_controllers_library/steering_controllers_library.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

using ControllerTwistReferenceMsg =
  steering_controllers_library::SteeringControllersLibrary::ControllerTwistReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerTwistReferenceMsg> & msg,
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

namespace steering_controllers_library
{
SteeringControllersLibrary::SteeringControllersLibrary()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn SteeringControllersLibrary::on_init()
{
  try
  {
    param_listener_ = std::make_shared<steering_controllers_library::ParamListener>(get_node());
    initialize_implementation_parameter_listener();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringControllersLibrary::set_interface_numbers(
  size_t nr_state_itfs = 2, size_t nr_cmd_itfs = 2, size_t nr_ref_itfs = 2)
{
  nr_state_itfs_ = nr_state_itfs;
  nr_cmd_itfs_ = nr_cmd_itfs;
  nr_ref_itfs_ = nr_ref_itfs;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringControllersLibrary::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // TODO(anyone): Remove deprecated parameters
  // START OF DEPRECATED
  if (!params_.front_steering)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'front_steering'. Instead, set 'traction_joints_names' or "
      "'steering_joints_names'");
  }

  if (params_.front_wheels_names.size() > 0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'front_wheels_names', set 'traction_joints_names' or "
      "'steering_joints_names' instead");
    if (params_.front_steering)
    {
      params_.steering_joints_names = params_.front_wheels_names;
    }
    else
    {
      params_.traction_joints_names = params_.front_wheels_names;
    }
  }

  if (params_.rear_wheels_names.size() > 0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'rear_wheels_names', set 'traction_joints_names' or "
      "'steering_joints_names' instead");
    if (params_.front_steering)
    {
      params_.traction_joints_names = params_.rear_wheels_names;
    }
    else
    {
      params_.steering_joints_names = params_.rear_wheels_names;
    }
  }

  if (params_.front_wheels_state_names.size() > 0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'front_wheels_state_names', set 'traction_joints_state_names' or "
      "'steering_joints_state_names' instead");
    if (params_.front_steering)
    {
      params_.steering_joints_state_names = params_.front_wheels_state_names;
    }
    else
    {
      params_.traction_joints_state_names = params_.front_wheels_state_names;
    }
  }

  if (params_.rear_wheels_state_names.size() > 0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'rear_wheels_state_names', set 'traction_joints_state_names' or "
      "'steering_joints_state_names' instead");
    if (params_.front_steering)
    {
      params_.traction_joints_state_names = params_.rear_wheels_state_names;
    }
    else
    {
      params_.steering_joints_state_names = params_.rear_wheels_state_names;
    }
  }
  // END OF DEPRECATED

  // call method from implementations, sets odometry type
  configure_odometry();

  // Check if the number of traction joints is correct
  if (odometry_.get_odometry_type() == steering_odometry::BICYCLE_CONFIG)
  {
    if (params_.traction_joints_names.size() != 1)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Bicycle configuration requires exactly one traction joint, but %zu were provided",
        params_.traction_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else if (odometry_.get_odometry_type() == steering_odometry::TRICYCLE_CONFIG)
  {
    if (params_.traction_joints_names.size() != 2)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Tricycle configuration requires exactly two traction joints, but %zu were provided",
        params_.traction_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else if (odometry_.get_odometry_type() == steering_odometry::ACKERMANN_CONFIG)
  {
    if (params_.traction_joints_names.size() != 2)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Ackermann configuration requires exactly two traction joints, but %zu were provided",
        params_.traction_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  // Check if the number of steering joints is correct
  if (odometry_.get_odometry_type() == steering_odometry::BICYCLE_CONFIG)
  {
    if (params_.steering_joints_names.size() != 1)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Bicycle configuration requires exactly one steering joint, but %zu were provided",
        params_.steering_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else if (odometry_.get_odometry_type() == steering_odometry::TRICYCLE_CONFIG)
  {
    if (params_.steering_joints_names.size() != 1)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Tricycle configuration requires exactly one steering joint, but %zu were provided",
        params_.steering_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else if (odometry_.get_odometry_type() == steering_odometry::ACKERMANN_CONFIG)
  {
    if (params_.steering_joints_names.size() != 2)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Ackermann configuration requires exactly two steering joints, but %zu were provided",
        params_.steering_joints_names.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  odometry_.set_velocity_rolling_window_size(
    static_cast<size_t>(params_.velocity_rolling_window_size));

  if (!params_.traction_joints_state_names.empty())
  {
    if (odometry_.get_odometry_type() == steering_odometry::BICYCLE_CONFIG)
    {
      if (params_.traction_joints_state_names.size() != 1)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Bicycle configuration requires exactly one traction joint, but %zu state interface "
          "names were provided",
          params_.traction_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else if (odometry_.get_odometry_type() == steering_odometry::TRICYCLE_CONFIG)
    {
      if (params_.traction_joints_state_names.size() != 2)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Tricycle configuration requires exactly two traction joints, but %zu state interface "
          "names were provided",
          params_.traction_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else if (odometry_.get_odometry_type() == steering_odometry::ACKERMANN_CONFIG)
    {
      if (params_.traction_joints_state_names.size() != 2)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Ackermann configuration requires exactly two traction joints, but %zu state interface "
          "names were provided",
          params_.traction_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    traction_joints_state_names_ = params_.traction_joints_state_names;
  }
  else
  {
    traction_joints_state_names_ = params_.traction_joints_names;
  }

  if (!params_.steering_joints_state_names.empty())
  {
    if (odometry_.get_odometry_type() == steering_odometry::BICYCLE_CONFIG)
    {
      if (params_.steering_joints_state_names.size() != 1)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Bicycle configuration requires exactly one steering joint, but %zu state interface "
          "names were provided",
          params_.steering_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else if (odometry_.get_odometry_type() == steering_odometry::TRICYCLE_CONFIG)
    {
      if (params_.steering_joints_state_names.size() != 1)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Tricycle configuration requires exactly one steering joint, but %zu state interface "
          "names were provided",
          params_.steering_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    else if (odometry_.get_odometry_type() == steering_odometry::ACKERMANN_CONFIG)
    {
      if (params_.steering_joints_state_names.size() != 2)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Ackermann configuration requires exactly two steering joints, but %zu state interface "
          "names were provided",
          params_.steering_joints_state_names.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    steering_joints_state_names_ = params_.steering_joints_state_names;
  }
  else
  {
    steering_joints_state_names_ = params_.steering_joints_names;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  ref_subscriber_twist_ = get_node()->create_subscription<ControllerTwistReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&SteeringControllersLibrary::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerTwistReferenceMsg> msg =
    std::make_shared<ControllerTwistReferenceMsg>();
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
    controller_s_publisher_ = get_node()->create_publisher<SteeringControllerStateMsg>(
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

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->unlock();
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void SteeringControllersLibrary::reference_callback(
  const std::shared_ptr<ControllerTwistReferenceMsg> msg)
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

controller_interface::InterfaceConfiguration
SteeringControllersLibrary::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);
  for (size_t i = 0; i < params_.traction_joints_names.size(); i++)
  {
    command_interfaces_config.names.push_back(
      params_.traction_joints_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (size_t i = 0; i < params_.steering_joints_names.size(); i++)
  {
    command_interfaces_config.names.push_back(
      params_.steering_joints_names[i] + "/" + hardware_interface::HW_IF_POSITION);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
SteeringControllersLibrary::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(nr_state_itfs_);
  const auto traction_wheels_feedback = params_.position_feedback
                                          ? hardware_interface::HW_IF_POSITION
                                          : hardware_interface::HW_IF_VELOCITY;

  for (size_t i = 0; i < traction_joints_state_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      traction_joints_state_names_[i] + "/" + traction_wheels_feedback);
  }

  for (size_t i = 0; i < steering_joints_state_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      steering_joints_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
SteeringControllersLibrary::on_export_reference_interfaces()
{
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

bool SteeringControllersLibrary::on_set_chained_mode(bool chained_mode)
{
  // Fix to adhere to CppLint standards
  (void)chained_mode;
  return true;
}

controller_interface::CallbackReturn SteeringControllersLibrary::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringControllersLibrary::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < nr_cmd_itfs_; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SteeringControllersLibrary::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = *(input_ref_.readFromRT());

  if (!std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.angular.z))
  {
    reference_interfaces_[0] = current_ref->twist.linear.x;
    reference_interfaces_[1] = current_ref->twist.angular.z;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type SteeringControllersLibrary::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  update_odometry(period);

  // MOVE ROBOT

  // Limit velocities and accelerations:
  // TODO(destogl): add limiter for the velocities

  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]))
  {
    const auto age_of_last_command = time - (*(input_ref_.readFromRT()))->header.stamp;
    const auto timeout =
      age_of_last_command > ref_timeout_ && ref_timeout_ != rclcpp::Duration::from_seconds(0);

    // store (for open loop odometry) and set commands
    last_linear_velocity_ = timeout ? 0.0 : reference_interfaces_[0];
    last_angular_velocity_ = timeout ? 0.0 : reference_interfaces_[1];

    auto [traction_commands, steering_commands] = odometry_.get_commands(
      reference_interfaces_[0], reference_interfaces_[1], params_.open_loop,
      params_.reduce_wheel_speed_until_steering_reached);

    for (size_t i = 0; i < params_.traction_joints_names.size(); i++)
    {
      command_interfaces_[i].set_value(timeout ? 0.0 : traction_commands[i]);
    }
    for (size_t i = 0; i < params_.steering_joints_names.size(); i++)
    {
      command_interfaces_[i + params_.traction_joints_names.size()].set_value(steering_commands[i]);
    }
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
    controller_state_publisher_->msg_.traction_wheels_position.clear();
    controller_state_publisher_->msg_.traction_wheels_velocity.clear();
    controller_state_publisher_->msg_.linear_velocity_command.clear();
    controller_state_publisher_->msg_.steer_positions.clear();
    controller_state_publisher_->msg_.steering_angle_command.clear();

    auto number_of_traction_wheels = params_.traction_joints_names.size();
    auto number_of_steering_wheels = params_.steering_joints_names.size();

    for (size_t i = 0; i < number_of_traction_wheels; ++i)
    {
      if (params_.position_feedback)
      {
        controller_state_publisher_->msg_.traction_wheels_position.push_back(
          state_interfaces_[i].get_value());
      }
      else
      {
        controller_state_publisher_->msg_.traction_wheels_velocity.push_back(
          state_interfaces_[i].get_value());
      }
      controller_state_publisher_->msg_.linear_velocity_command.push_back(
        command_interfaces_[i].get_value());
    }

    for (size_t i = 0; i < number_of_steering_wheels; ++i)
    {
      controller_state_publisher_->msg_.steer_positions.push_back(
        state_interfaces_[number_of_traction_wheels + i].get_value());
      controller_state_publisher_->msg_.steering_angle_command.push_back(
        command_interfaces_[number_of_traction_wheels + i].get_value());
    }

    controller_state_publisher_->unlockAndPublish();
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

}  // namespace steering_controllers_library
