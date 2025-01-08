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

#include "inverse_kinematics.hpp"

std::pair<std::vector<double>, std::vector<double>> InverseKinematics::calculateCommands(
  double linear_velocity, double angular_velocity)
{
  std::vector<double> traction_commands(params_.rear_wheels_names.size(), 0.0);
  std::vector<double> steering_commands(params_.front_wheels_names.size(), 0.0);

  double wheel_base = params_.wheel_base;
  double track_width = params_.track_width;

  double radius = (angular_velocity != 0.0) ? (linear_velocity / angular_velocity)
                                            : std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < traction_commands.size(); ++i)
  {
    traction_commands[i] = linear_velocity;
  }

  for (size_t i = 0; i < steering_commands.size(); ++i)
  {
    steering_commands[i] = std::atan2(wheel_base, radius);
  }

  return std::make_pair(traction_commands, steering_commands);
}
