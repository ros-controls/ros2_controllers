// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef ACKERMANN_STEERING_CONTROLLER__HPP_
#define ACKERMANN_STEERING_CONTROLLER___HPP_

#include "steering_controllers/steering_controllers.hpp"

namespace ackermann_steering_controller
{
class AckermannSteeringController : public steering_controllers::SteeringControllers
{
public:
  AckermannSteeringController();

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn configure_odometry()
    override;
};
}  // namespace ackermann_steering_controller

namespace bicycle_steering_controller
{
class BicycleSteeringController : public steering_controllers::SteeringControllers
{
public:
  BicycleSteeringController();

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn configure_odometry()
    override;
};
}  // namespace bicycle_steering_controller

#endif  // ACKERMANN_STEERING_CONTROLLER__HPP_
