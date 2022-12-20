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
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

#ifndef STEERING_CONTROLLERS_IMPLEMENTATIONS_HPP_
#define STEERING_CONTROLLERS_IMPLEMENTATIONS_HPP_

#include "steering_controllers/steering_controllers.hpp"

namespace tricycle_steering_controller
{
class TricycleSteeringController : public steering_controllers::SteeringControllers
{
public:
  TricycleSteeringController();

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn configure_odometry()
    override;
  STEERING_CONTROLLERS__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) override;
};
}  // namespace tricycle_steering_controller

#endif  // STEERING_CONTROLLERS_IMPLEMENTATIONS_HPP_
