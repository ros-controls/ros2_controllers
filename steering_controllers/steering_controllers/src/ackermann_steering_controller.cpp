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
//using AckermannSteeringController = steering_controllers::SteeringControllers;
AckermannSteeringController::AckermannSteeringController()
: steering_controllers::SteeringControllers()
{
}

controller_interface::CallbackReturn AckermannSteeringController::configure_odometry()
{
  params_ = param_listener_->get_params();

  const double wheel_seperation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheel_radius = params_.wheel_radius_multiplier * params_.wheel_radius;
  const double wheelbase = params_.wheelbase_multiplier * params_.wheelbase;
  odometry_.set_wheel_params(wheel_seperation, wheel_radius, wheelbase);
  odometry_.set_velocity_rolling_window_size(params_.velocity_rolling_window_size);

  const size_t nr_state_itfs = 2;
  const size_t nr_cmd_itfs = 2;
  const size_t nr_ref_itfs = 2;

  set_interface_numbers(nr_state_itfs, nr_cmd_itfs, nr_ref_itfs);

  RCLCPP_INFO(get_node()->get_logger(), "odom configure successful INSIDE ACKERMANN");
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ackermann_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ackermann_steering_controller::AckermannSteeringController,
  controller_interface::ChainableControllerInterface)
