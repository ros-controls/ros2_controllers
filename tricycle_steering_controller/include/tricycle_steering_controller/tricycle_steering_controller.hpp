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

#ifndef TRICYCLE_STEERING_CONTROLLER__TRICYCLE_STEERING_CONTROLLER_HPP_
#define TRICYCLE_STEERING_CONTROLLER__TRICYCLE_STEERING_CONTROLLER_HPP_

#include <memory>

#include "steering_controllers_library/steering_controllers_library.hpp"
#include "tricycle_steering_controller/tricycle_steering_controller_parameters.hpp"

namespace tricycle_steering_controller
{
// name constants for state interfaces
static constexpr size_t STATE_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t STATE_DUAL_TRACTION_STEER_AXIS = 2;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t CMD_DUAL_TRACTION_STEER_WHEEL = 2;

// name constants for state interfaces
static constexpr size_t STATE_TRACTION_SINGLE_WHEEL = 0;
static constexpr size_t STATE_SINGLE_TRACTION_STEER_AXIS = 1;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_SINGLE_WHEEL = 0;
static constexpr size_t CMD_SINGLE_TRACTION_STEER_WHEEL = 1;

class TricycleSteeringController : public steering_controllers_library::SteeringControllersLibrary
{
public:
  TricycleSteeringController();

  controller_interface::CallbackReturn configure_odometry() override;

  bool update_odometry(const rclcpp::Duration & period) override;

  void initialize_implementation_parameter_listener() override;

protected:
  std::shared_ptr<tricycle_steering_controller::ParamListener> tricycle_param_listener_;
  tricycle_steering_controller::Params tricycle_params_;
};
}  // namespace tricycle_steering_controller

#endif  // TRICYCLE_STEERING_CONTROLLER__TRICYCLE_STEERING_CONTROLLER_HPP_
