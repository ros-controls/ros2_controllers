// Copyright 2020 PAL Robotics S.L.
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

#ifndef DIFF_DRIVE_CONTROLLER__URDF_UTIL_HPP_
#define DIFF_DRIVE_CONTROLLER__URDF_UTIL_HPP_

#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>
#include <rclcpp/logging.hpp>

namespace urdf_util
{
/**
 * \brief Calculate L2 distance between two vectors
 * \param vec1 first vector
 * \param vec2 second vector
 * \return distance between the two
 */
double euclidean_of_vectors(const urdf::Vector3 & vec1, const urdf::Vector3 & vec2);

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
bool get_wheel_radius(
  const urdf::LinkConstSharedPtr & wheel_link, double & wheel_radius,
  const rclcpp::Logger & logger);
}  // namespace urdf_util

#endif  // DIFF_DRIVE_CONTROLLER__URDF_UTIL_HPP_
