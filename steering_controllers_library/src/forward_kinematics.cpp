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

#include "forward_kinematics.hpp"

struct Odometry
{
  double x;
  double y;
  double theta;
};

Odometry ForwardKinematics::calculate(double linear_velocity, double angular_velocity)
{
  Odometry odom_result;
  odom_result.x = linear_velocity * params_.wheel_base;
  odom_result.y = 0.0;
  odom_result.theta = angular_velocity;

  return odom_result;
}
