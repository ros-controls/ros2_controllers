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

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

#include "steering_controllers_library/steering_kinematics.hpp"

// Deprecation notice
#ifdef _WIN32
#pragma message("This header is deprecated. Use 'steering_kinematics.hpp' instead.")  // NOLINT
#else
#warning "This header is deprecated. Use 'steering_kinematics.hpp' instead."  // NOLINT
#endif

namespace steering_odometry
{
using SteeringOdometry [[deprecated("Use steering_kinematics::SteeringKinematics instead")]] =
  steering_kinematics::SteeringKinematics;
}

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
// Create an alias for backward compatibility
