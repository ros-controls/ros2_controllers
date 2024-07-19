// Copyright (c) 2022 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace joint_trajectory_controller
{
static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("joint_trajectory_controller.interpolation_methods");

namespace interpolation_methods
{
enum class InterpolationMethod
{
  NONE,
  VARIABLE_DEGREE_SPLINE
};

const InterpolationMethod DEFAULT_INTERPOLATION = InterpolationMethod::VARIABLE_DEGREE_SPLINE;

const std::unordered_map<InterpolationMethod, std::string> InterpolationMethodMap(
  {{InterpolationMethod::NONE, "none"}, {InterpolationMethod::VARIABLE_DEGREE_SPLINE, "splines"}});

[[nodiscard]] inline InterpolationMethod from_string(const std::string & interpolation_method)
{
  if (interpolation_method.compare(InterpolationMethodMap.at(InterpolationMethod::NONE)) == 0)
  {
    return InterpolationMethod::NONE;
  }
  else if (
    interpolation_method.compare(
      InterpolationMethodMap.at(InterpolationMethod::VARIABLE_DEGREE_SPLINE)) == 0)
  {
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
  // Default
  else
  {
    RCLCPP_INFO_STREAM(
      LOGGER,
      "No interpolation method parameter was given. Using the default, VARIABLE_DEGREE_SPLINE.");
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
}
}  // namespace interpolation_methods
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
