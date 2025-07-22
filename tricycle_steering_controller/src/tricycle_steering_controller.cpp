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

#include "tricycle_steering_controller/tricycle_steering_controller.hpp"

namespace tricycle_steering_controller
{
TricycleSteeringController::TricycleSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}
void TricycleSteeringController::initialize_implementation_parameter_listener()
{
  tricycle_param_listener_ =
    std::make_shared<tricycle_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn TricycleSteeringController::configure_odometry()
{
  tricycle_params_ = tricycle_param_listener_->get_params();

  const double traction_wheels_radius = tricycle_params_.traction_wheels_radius;
  const double traction_track_width = tricycle_params_.traction_track_width;
  const double wheelbase = tricycle_params_.wheelbase;

  odometry_.set_wheel_params(traction_wheels_radius, wheelbase, traction_track_width);
  odometry_.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "tricycle odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool TricycleSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const auto traction_right_wheel_value_op =
      state_interfaces_[STATE_TRACTION_RIGHT_WHEEL].get_optional();
    const auto traction_left_wheel_value_op =
      state_interfaces_[STATE_TRACTION_LEFT_WHEEL].get_optional();
    const auto steering_position_op = state_interfaces_[STATE_STEER_AXIS].get_optional();

    if (
      !traction_right_wheel_value_op.has_value() || !traction_left_wheel_value_op.has_value() ||
      !steering_position_op.has_value())
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Unable to retrieve the data from right wheel or left wheel or steering position");
      return true;
    }
    const double traction_right_wheel_value = traction_right_wheel_value_op.value();
    const double traction_left_wheel_value = traction_left_wheel_value_op.value();
    const double steering_position = steering_position_op.value();
    if (
      std::isfinite(traction_right_wheel_value) && std::isfinite(traction_left_wheel_value) &&
      std::isfinite(steering_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          traction_right_wheel_value, traction_left_wheel_value, steering_position,
          period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          traction_right_wheel_value, traction_left_wheel_value, steering_position,
          period.seconds());
      }
    }
  }
  return true;
}

}  // namespace tricycle_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tricycle_steering_controller::TricycleSteeringController,
  controller_interface::ChainableControllerInterface)
