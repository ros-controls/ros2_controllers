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

#ifndef FORWARD_KINEMATICS_HPP_
#define FORWARD_KINEMATICS_HPP_

#include "steering_controllers_library_parameters.hpp"

class ForwardKinematics
{
public:
  explicit ForwardKinematics(const steering_controllers_library::Params & params) : params_(params)
  {
  }
  Odometry calculate(
    double linear_velocity, double angular_velocity);  // Implement forward calculation

private:
  steering_controllers_library::Params params_;
};

#endif  // FORWARD_KINEMATICS_HPP_
