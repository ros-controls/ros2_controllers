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

#include <cctype>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace joint_trajectory_controller
{

namespace
{

/// Converts a string to lowercase for case-agnostic checking.
inline std::string convert_to_lowercase(const std::string & str)
{
  std::string s = str;
  for (char & c : s)
  {
    // C++ std requires the argument passed to std::tolower must be representable as
    // unsigned char or equal to EOF.
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

}  //  namespace

/// \brief Setup interpolation_methods' rclcpp::Logger instance.
static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("joint_trajectory_controller.interpolation_methods");

namespace interpolation_methods
{

/**
 * \brief Defines the available interpolation methods used for fitting data curves.
 * This enumeration specifies how intermediate values between data points
 * should be calculated.
 */
enum class InterpolationMethod
{
  /**
   * \brief No interpolation is performed.
   * This is typically used when data points are discrete and should not be
   * connected by a curve.
   */
  NONE,

  /**
   * \brief Uses a variable-degree spline interpolation.
   * The degree of the spline is determined dynamically based on the number of
   * available deriviatives. This provides a smooth, continuous curve between data points.
   *
   * Based on available deriviatives, it uses following degree interpolation,
   * 1. Neither velocity nor acceleration is available: `Linear Interpolation`.
   * 2. Velocity is available, but acceleration is not available: `Cubic Spline Interpolation`.
   * 3. Both velocity and acceleration is available: `Quintic Spline Interpolation`.
   */
  VARIABLE_DEGREE_SPLINE
};

/**
 * \brief The default interpolation method is set to `InterpolationMethod::VARIABLE_DEGREE_SPLINE`.
 * As, it provides most realistic, jerk-free and smooth motion.
 */
const InterpolationMethod DEFAULT_INTERPOLATION = InterpolationMethod::VARIABLE_DEGREE_SPLINE;

/**
 * \brief Maps `InterpolationMethod` enum values to their string identifiers.
 * This constant map is used to look up the InterpolationMethod for a given
 * string (e.g., "splines" for `VARIABLE_DEGREE_SPLINE`).
 */
const std::unordered_map<std::string, InterpolationMethod> InterpolationMethodMap(
  {{"none", InterpolationMethod::NONE}, {"splines", InterpolationMethod::VARIABLE_DEGREE_SPLINE}});

/**
 * \brief Creates a `InterpolationMethod` enum class value from a string.
 * This function looks up `InterpolationMethodMap` for corresponding `InterpolationMethod` based
 * on interpolation_method string.
 *
 * \param[in] interpolation_method The given interpolation method string.
 *
 * \returns The corresponding InterpolationMethod.
 *
 * \note If interpolation_method does not have any corresponding InterpolationMethod
 * (i.e., "Unknown"), it defaults to `InterpolationMethod::VARIABLE_DEGREE_SPLINE`.
 */
[[nodiscard]] inline InterpolationMethod from_string(const std::string & interpolation_method)
{
  // Convert to lowercase, so we have an case-agnostic checking,
  // (i.e., None and none, etc are treated same.)
  std::string method = convert_to_lowercase(interpolation_method);

  // Iterator to InterpolationMethodMap
  const auto iterator = InterpolationMethodMap.find(method);

  // If interpolation_method exists
  if (iterator != InterpolationMethodMap.end())
  {
    // Return corresponding `InterpolationMethod`
    return iterator->second;
  }
  // Default
  else
  {
    RCLCPP_WARN(
      LOGGER,
      "Unknown interpolation method parameter '%s' was given. Using the default: "
      "VARIABLE_DEGREE_SPLINE.",
      interpolation_method.c_str());
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
}

/**
 * \brief Returns corresponding string value for the `InterpolationMethod`.
 * This function uses simple switch-case lookup to directly assign string value to
 * `InterpolationMethod`.
 *
 * \param[in] interpolation_method The InterpolationMethod enum value.
 *
 * \returns The corresponding string.
 *
 * \note Defaults to return "UNKNOWN".
 */
[[nodiscard]] inline std::string to_string(const InterpolationMethod & interpolation_method)
{
  switch (interpolation_method)
  {
    case InterpolationMethod::NONE:
      return "none";
    case InterpolationMethod::VARIABLE_DEGREE_SPLINE:
      return "splines";
    // Default
    default:
      RCLCPP_WARN(
        LOGGER,
        "Unknown interpolation method enum value was given. Returing default: "
        "UNKNOWN");
      return "UNKNOWN";
  }
}

}  // namespace interpolation_methods
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
