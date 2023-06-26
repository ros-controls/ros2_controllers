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

#include "bicycle_steering_controller/bicycle_steering_controller.hpp"

namespace bicycle_steering_controller
{
BicycleSteeringController::BicycleSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void BicycleSteeringController::initialize_implementation_parameter_listener()
{
  bicycle_param_listener_ =
    std::make_shared<bicycle_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn BicycleSteeringController::configure_odometry()
{
  bicycle_params_ = bicycle_param_listener_->get_params();

  const double wheelbase = bicycle_params_.wheelbase;
  const double front_wheel_radius = bicycle_params_.front_wheel_radius;
  const double rear_wheel_radius = bicycle_params_.rear_wheel_radius;

  if (params_.front_steering)
  {
    odometry_.set_wheel_params(rear_wheel_radius, wheelbase);
  }
  else
  {
    odometry_.set_wheel_params(front_wheel_radius, wheelbase);
  }

  odometry_.set_odometry_type(steering_odometry::BICYCLE_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "bicycle odometry configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BicycleSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double rear_wheel_value = state_interfaces_[STATE_TRACTION_WHEEL].get_value();
    const double steer_position = state_interfaces_[STATE_STEER_AXIS].get_value();
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
  return true;
}
}  // namespace bicycle_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bicycle_steering_controller::BicycleSteeringController,
  controller_interface::ChainableControllerInterface)
