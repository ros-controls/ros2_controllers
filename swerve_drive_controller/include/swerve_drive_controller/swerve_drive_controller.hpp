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
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVEN_STEERING_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVEN_STEERING_CONTROLLER_HPP_

#include <memory>

#include "swerve_drive_controller/visibility_control.h"
#include "swerve_drive_controller_parameters.hpp"
#include "steering_controllers_library/steering_controllers_library.hpp"

namespace swerve_drive_controller
{
// name constants for state interfaces
static constexpr size_t STATE_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t STATE_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t STATE_TRACTION_REAR_LEFT_WHEEL = 3;
static constexpr size_t STATE_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t STATE_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t STATE_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t STATE_STEER_REAR_LEFT_WHEEL = 7;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t CMD_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t CMD_TRACTION_REAR_LEFT_WHEEL = 3;
static constexpr size_t CMD_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t CMD_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t CMD_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t CMD_STEER_REAR_LEFT_WHEEL = 7;

static constexpr size_t NR_STATE_ITFS = 8;
static constexpr size_t NR_CMD_ITFS = 8;
static constexpr size_t NR_REF_ITFS = 2;

class SwerveDriveController : public steering_controllers_library::SteeringControllersLibrary
{
public:
  SwerveDriveController();

  SWERVE_DRIVE_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry() override;

  SWERVE_DRIVE_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) override;

  SWERVE_DRIVE_CONTROLLER__VISIBILITY_PUBLIC void
  initialize_implementation_parameter_listener() override;

protected:
  std::shared_ptr<swerve_drive_controller::ParamListener> swerve_param_listener_;
  swerve_drive_controller::Params swerve_params_;
};
}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVEN_STEERING_CONTROLLER_HPP_
