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

  const double front_wheels_radius = ackermann_params_.front_wheels_radius;
  const double rear_wheels_radius = ackermann_params_.rear_wheels_radius;
  const double front_wheel_track = ackermann_params_.front_wheel_track;
  const double rear_wheel_track = ackermann_params_.rear_wheel_track;
  const double wheelbase = ackermann_params_.wheelbase;

  if (params_.front_steering)
  {
    odometry_.set_wheel_params(rear_wheels_radius, wheelbase, rear_wheel_track);
  }
  else
  {
    odometry_.set_wheel_params(front_wheels_radius, wheelbase, front_wheel_track);
  }

  odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "ackermann odom configure successful");
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
