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

#include "swerve_drive_controller/swerve_drive_controller.hpp"

namespace swerve_drive_controller
{
SwerveDriveController::SwerveDriveController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void SwerveDriveController::initialize_implementation_parameter_listener()
{
  swerve_param_listener_ =
    std::make_shared<swerve_drive_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn SwerveDriveController::configure_odometry()
{
  swerve_params_ = swerve_param_listener_->get_params();

  const double wheels_radius = swerve_params_.wheels_radius;
  const double wheel_track = swerve_params_.wheel_track;
  const double wheelbase = swerve_params_.wheelbase;

  odometry_.set_wheel_params(wheels_radius, wheelbase, wheel_track);

  odometry_.set_odometry_type(steering_odometry::SWERVE_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "swerve odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SwerveDriveController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {

    //TODO: Take into account REAR wheels and steering joints
    const double front_right_wheel_value = state_interfaces_[STATE_TRACTION_FRONT_RIGHT_WHEEL].get_value();
    const double front_left_wheel_value = state_interfaces_[STATE_TRACTION_FRONT_LEFT_WHEEL].get_value();
    const double front_right_steer_position =
      state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
    const double front_left_steer_position = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
    if (
      !std::isnan(front_right_wheel_value) && !std::isnan(front_left_wheel_value) &&
      !std::isnan(front_right_steer_position) && !std::isnan(front_left_steer_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          front_right_wheel_value, front_left_wheel_value, front_right_steer_position,
          front_left_steer_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          front_right_wheel_value, front_left_wheel_value, front_right_steer_position,
          front_left_steer_position, period.seconds());
      }
    }
  }
  return true;
}
}  // namespace swerve_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  swerve_drive_controller::SwerveDriveController,
  controller_interface::ChainableControllerInterface)
