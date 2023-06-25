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

#include "four_wheel_steering_controller/four_wheel_steering_controller.hpp"

namespace four_wheel_steering_controller
{
FourWheelSteeringController::FourWheelSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void FourWheelSteeringController::initialize_implementation_parameter_listener()
{
  four_wheel_steering_param_listener_ =
    std::make_shared<four_wheel_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn FourWheelSteeringController::configure_odometry()
{
  four_wheel_steering_param_ = four_wheel_steering_param_listener_->get_params();

  const double wheel_radius = four_wheel_steering_param_.wheel_radius;
  const double wheel_track = four_wheel_steering_param_.wheel_track;
  const double wheelbase = four_wheel_steering_param_.wheelbase;
  const double y_steering_offset = four_wheel_steering_param_.y_steering_offset;

  odometry_.set_wheel_params(wheel_radius, wheelbase, wheel_track, y_steering_offset);

  odometry_.set_odometry_type(steering_odometry::FOUR_WHEEL_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "four_wheel odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool FourWheelSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double front_right_wheel_value = state_interfaces_[STATE_TRACTION_FRONT_RIGHT_WHEEL].get_value();
    const double front_left_wheel_value = state_interfaces_[STATE_TRACTION_FRONT_LEFT_WHEEL].get_value();
    const double rear_right_wheel_value = state_interfaces_[STATE_TRACTION_REAR_RIGHT_WHEEL].get_value();
    const double rear_left_wheel_value = state_interfaces_[STATE_TRACTION_REAR_LEFT_WHEEL].get_value();

    const double front_right_steer_position = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
    const double front_left_steer_position = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
    const double rear_right_steer_position = state_interfaces_[STATE_STEER_REAR_RIGHT_WHEEL].get_value();
    const double rear_left_steer_position = state_interfaces_[STATE_STEER_REAR_LEFT_WHEEL].get_value();

    if (
      !std::isnan(front_right_wheel_value) && !std::isnan(front_left_wheel_value) &&
      !std::isnan(rear_right_wheel_value) && !std::isnan(rear_left_wheel_value) &&
      !std::isnan(front_right_steer_position) && !std::isnan(front_left_steer_position) &&
      !std::isnan(rear_right_steer_position) && !std::isnan(rear_left_steer_position))
    {
      if (params_.position_feedback)
      {
        double front_steer_position = 0.0;
        if(fabs(front_right_steer_position) > 0.001 || fabs(front_left_steer_position) > 0.001)
        {
          front_steer_position = atan(2 * tan(front_right_steer_position) * tan(front_left_steer_position) /
                                          (tan(front_right_steer_position) + tan(front_left_steer_position)));
        }
        double rear_steer_position = 0.0;
        if(fabs(rear_right_steer_position) > 0.001 || fabs(rear_left_steer_position) > 0.001)
        {
          rear_steer_position = atan(2*tan(rear_right_steer_position)*tan(rear_left_steer_position)/
                                        (tan(rear_right_steer_position) + tan(rear_left_steer_position)));
        }
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          front_right_wheel_value, front_left_wheel_value, rear_right_wheel_value,
          rear_left_wheel_value, front_steer_position, rear_steer_position period.seconds());
    }
  }
  return true;
}
}  // namespace four_wheel_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  four_wheel_steering_controller::FourWheelSteeringController,
  controller_interface::ChainableControllerInterface)
