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

#include "steering_controllers/tricycle_steering_controller.hpp"

namespace tricycle_steering_controller
{
TricycleSteeringController::TricycleSteeringController()
: steering_controllers::SteeringControllers()
{
}

controller_interface::CallbackReturn TricycleSteeringController::configure_odometry()
{
  const double wheel_radius = params_.wheel_radius_multiplier * params_.wheel_radius;
  const double wheel_seperation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheelbase = params_.wheelbase_multiplier * params_.wheelbase;
  odometry_.set_wheel_params(wheel_radius, wheel_seperation, wheelbase);
  odometry_.set_velocity_rolling_window_size(params_.velocity_rolling_window_size);

  // TODO: enable position/velocity configure
  const size_t nr_state_itfs = 3;
  const size_t nr_cmd_itfs = 3;
  const size_t nr_ref_itfs = 2;

  set_interface_numbers(nr_state_itfs, nr_cmd_itfs, nr_ref_itfs);

  RCLCPP_INFO(get_node()->get_logger(), "ackermann odom configure successful");
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
    const double rear_right_wheel_value = state_interfaces_[0].get_value();
    const double rear_left_wheel_value = state_interfaces_[1].get_value();
    const double steer_position = state_interfaces_[2].get_value() * params_.steer_pos_multiplier;
    if (
      !std::isnan(rear_right_wheel_value) && !std::isnan(rear_left_wheel_value) &&
      !std::isnan(steer_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          rear_right_wheel_value, rear_left_wheel_value, steer_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          rear_right_wheel_value, rear_left_wheel_value, steer_position, period.seconds());
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
