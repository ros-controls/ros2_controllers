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

#ifndef ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_
#define ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_

#include <memory>

#include "ackermann_steering_controller_parameters.hpp"
#include "steering_controllers_library/steering_controllers_library.hpp"

namespace ackermann_steering_controller
{
// name constants for state interfaces
static constexpr size_t STATE_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t STATE_STEER_RIGHT_WHEEL = 2;
static constexpr size_t STATE_STEER_LEFT_WHEEL = 3;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t CMD_STEER_RIGHT_WHEEL = 2;
static constexpr size_t CMD_STEER_LEFT_WHEEL = 3;

static constexpr size_t NR_STATE_ITFS = 4;
static constexpr size_t NR_CMD_ITFS = 4;
static constexpr size_t NR_REF_ITFS = 2;

class AckermannSteeringController : public steering_controllers_library::SteeringControllersLibrary
{
public:
  AckermannSteeringController();

  controller_interface::CallbackReturn configure_odometry() override;

  bool update_odometry(const rclcpp::Duration & period) override;

  void initialize_implementation_parameter_listener() override;

protected:
  std::shared_ptr<ackermann_steering_controller::ParamListener> ackermann_param_listener_;
  ackermann_steering_controller::Params ackermann_params_;
};
}  // namespace ackermann_steering_controller

#endif  // ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_
