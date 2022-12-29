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

#include "steering_controllers/ackermann_steering_controller.hpp"

namespace ackermann_steering_controller
{
AckermannSteeringController::AckermannSteeringController()
: steering_controllers::SteeringControllers()
{
}

void AckermannSteeringController::initialize_implementation_parameter_listener()
{
  ackermann_param_listener_ =
    std::make_shared<ackermann_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn AckermannSteeringController::configure_odometry()
{
  ackermann_steering_controller::Params ackerman_params = ackermann_param_listener_->get_params();

  const double front_wheels_radius = ackerman_params.front_wheels_radius;
  const double rear_wheels_radius = ackerman_params.rear_wheels_radius;
  const double front_wheel_track = ackerman_params.front_wheel_track;
  const double rear_wheel_track = ackerman_params.rear_wheel_track;
  const double wheelbase = ackerman_params.wheelbase;

  if (params_.front_steering)
  {
    odometry_.set_wheel_params(rear_wheels_radius, wheelbase, rear_wheel_track);
  }
  else
  {
    odometry_.set_wheel_params(front_wheels_radius, wheelbase, front_wheel_track);
  }

  odometry_.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);

  const size_t nr_state_itfs = 4;
  const size_t nr_cmd_itfs = 4;
  const size_t nr_ref_itfs = 2;

  set_interface_numbers(nr_state_itfs, nr_cmd_itfs, nr_ref_itfs);

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
    const double rear_right_wheel_value = state_interfaces_[0].get_value();
    const double rear_left_wheel_value = state_interfaces_[1].get_value();
    const double front_right_steer_position = state_interfaces_[2].get_value();
    const double front_left_steer_position = state_interfaces_[3].get_value();
    if (
      !std::isnan(rear_right_wheel_value) && !std::isnan(rear_left_wheel_value) &&
      !std::isnan(front_right_steer_position) && !std::isnan(front_left_steer_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          rear_right_wheel_value, rear_left_wheel_value, front_right_steer_position,
          front_left_steer_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          rear_right_wheel_value, rear_left_wheel_value, front_right_steer_position,
          front_left_steer_position, period.seconds());
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
