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

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

#warning \
  "This header (steering_odometry.hpp) is deprecated. Please include steering_kinematics.hpp instead."
#include "steering_controllers_library/steering_kinematics.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace steering_odometry
{
[[deprecated("Use steering_kinematics::BICYCLE_CONFIG")]] const unsigned int BICYCLE_CONFIG =
  steering_kinematics::BICYCLE_CONFIG;
[[deprecated("Use steering_kinematics::TRICYCLE_CONFIG")]] const unsigned int TRICYCLE_CONFIG =
  steering_kinematics::TRICYCLE_CONFIG;
[[deprecated("Use steering_kinematics::ACKERMANN_CONFIG")]] const unsigned int ACKERMANN_CONFIG =
  steering_kinematics::ACKERMANN_CONFIG;

inline bool is_close_to_zero(double val) { return steering_kinematics::is_close_to_zero(val); }

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
using SteeringOdometry [[deprecated("Use steering_kinematics::SteeringKinematics instead.")]] =
  steering_kinematics::SteeringKinematics;

}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
