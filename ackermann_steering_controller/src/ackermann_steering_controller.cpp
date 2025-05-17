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

#include <limits>

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"

namespace ackermann_steering_controller
{
AckermannSteeringController::AckermannSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void AckermannSteeringController::initialize_implementation_parameter_listener()
{
  ackermann_param_listener_ =
    std::make_shared<ackermann_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn AckermannSteeringController::configure_odometry()
{
  ackermann_params_ = ackermann_param_listener_->get_params();

  // TODO(anyone): Remove deprecated parameters
  // START OF DEPRECATED
  if (ackermann_params_.front_wheels_radius > 0.0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'front_wheel_radius', set 'traction_wheels_radius' instead");
    ackermann_params_.traction_wheels_radius = ackermann_params_.front_wheels_radius;
  }

  if (ackermann_params_.rear_wheels_radius > 0.0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'rear_wheel_radius', set 'traction_wheels_radius' instead");
    ackermann_params_.traction_wheels_radius = ackermann_params_.rear_wheels_radius;
  }

  if (ackermann_params_.front_wheel_track > 0.0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'front_wheel_track', set 'traction_track_width' or "
      "'steering_track_width' instead");
    if (params_.front_steering)
    {
      ackermann_params_.steering_track_width = ackermann_params_.front_wheel_track;
    }
    else
    {
      ackermann_params_.traction_track_width = ackermann_params_.front_wheel_track;
    }
  }

  if (ackermann_params_.rear_wheel_track > 0.0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "DEPRECATED parameter 'rear_wheel_track', set 'traction_track_width' or "
      "'steering_track_width' instead");
    if (params_.front_steering)
    {
      ackermann_params_.traction_track_width = ackermann_params_.rear_wheel_track;
    }
    else
    {
      ackermann_params_.steering_track_width = ackermann_params_.rear_wheel_track;
    }
  }

  if (ackermann_params_.traction_wheels_radius <= std::numeric_limits<double>::epsilon())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "parameter 'traction_wheels_radius' is not set, cannot configure odometry");
    return controller_interface::CallbackReturn::ERROR;
  }
  // END OF DEPRECATED

  if (ackermann_params_.steering_track_width <= std::numeric_limits<double>::epsilon())
  {
    ackermann_params_.steering_track_width = ackermann_params_.traction_track_width;
  }

  const double traction_wheels_radius = ackermann_params_.traction_wheels_radius;
  const double traction_track_width = ackermann_params_.traction_track_width;
  const double steering_track_width = ackermann_params_.steering_track_width;
  const double wheelbase = ackermann_params_.wheelbase;

  odometry_.set_wheel_params(
    traction_wheels_radius, wheelbase, steering_track_width, traction_track_width);
  odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "Ackermann odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AckermannSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double traction_right_wheel_value =
      state_interfaces_[STATE_TRACTION_RIGHT_WHEEL].get_value();
    const double traction_left_wheel_value =
      state_interfaces_[STATE_TRACTION_LEFT_WHEEL].get_value();
    const double steering_right_position = state_interfaces_[STATE_STEER_RIGHT_WHEEL].get_value();
    const double steering_left_position = state_interfaces_[STATE_STEER_LEFT_WHEEL].get_value();
    if (
      std::isfinite(traction_right_wheel_value) && std::isfinite(traction_left_wheel_value) &&
      std::isfinite(steering_right_position) && std::isfinite(steering_left_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          traction_right_wheel_value, traction_left_wheel_value, steering_right_position,
          steering_left_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          traction_right_wheel_value, traction_left_wheel_value, steering_right_position,
          steering_left_position, period.seconds());
      }
    }
  }
  return true;
}
}  // namespace ackermann_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ackermann_steering_controller::AckermannSteeringController,
  controller_interface::ChainableControllerInterface)
