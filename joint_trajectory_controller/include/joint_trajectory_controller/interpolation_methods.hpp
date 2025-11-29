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

#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_trajectory_controller
{

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
   *
   * It returns the initial point until the time for the first trajectory data
   * point is reached. Then, it simply takes the next given datapoint.
   */
  NONE,

  /**
   * \brief Uses a variable-degree spline interpolation.
   * The degree of the spline is determined dynamically based on the number of
   * available derivatives. This provides a smooth, continuous curve between data points.
   *
   * Based on available derivatives, it uses the following degree interpolation,
   * 1. If only position is available: `Linear Interpolation`.
   * 2. If position, and velocity are available: `Cubic Spline Interpolation`.
   * 3. If position, velocity, and acceleration is available: `Quintic Spline Interpolation`.
   *
   * \note
   * `Linear Interpolation` is discouraged, due to it yields trajectories with discontinuous
   * velocities at the waypoints.
   */
  VARIABLE_DEGREE_SPLINE
};

/**
 * \brief The default interpolation method is set to `InterpolationMethod::VARIABLE_DEGREE_SPLINE`.
 * As it provides the most realistic, jerk-free and smooth motion.
 */
const InterpolationMethod DEFAULT_INTERPOLATION = InterpolationMethod::VARIABLE_DEGREE_SPLINE;

/**
 * @brief Maps string representations to their corresponding InterpolationMethod enum values.
 *
 * @deprecated This map is deprecated. Use the direct lookup methods instead.
 * (Original use: Converting strings into the InterpolationMethod).
 */
[[deprecated(
  "InterpolationMethodMap is expected to be removed in future iterations of JTC. "
  "Instead, use the direct lookup methods instead.")]]
const std::unordered_map<std::string, InterpolationMethod> InterpolationMethodMap(
  {{"none", InterpolationMethod::NONE}, {"splines", InterpolationMethod::VARIABLE_DEGREE_SPLINE}});

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
        "Unknown interpolation method enum value was given. Returning default: "
        "UNKNOWN");
      return "UNKNOWN";
  }
}

/**
 * \brief Creates a `InterpolationMethod` enum class value from a string.
 * This function directly looks up for corresponding `InterpolationMethod` based
 * on interpolation_method string (case-agnostic).
 *
 * \param[in] interpolation_method The given interpolation method string.
 *
 * \returns The corresponding InterpolationMethod.
 *
 * \note If interpolation_method does not have any corresponding InterpolationMethod
 * (i.e., "Unknown"), it defaults to `DEFAULT_INTERPOLATION`.
 */
[[nodiscard]] inline InterpolationMethod from_string(const std::string & interpolation_method)
{
  // Convert to lowercase, so we have a case-agnostic checking,
  // (i.e., "None, NONE, and none" are treated same.)
  std::string method = ::hardware_interface::to_lower_case(interpolation_method);

  if (method == "none")
  {
    return InterpolationMethod::NONE;
  }
  else if (method == "splines")
  {
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
  // Default
  else
  {
    RCLCPP_WARN(
      LOGGER,
      "Unknown interpolation method parameter '%s' was given. Using the default: "
      "DEFAULT_INTERPOLATION (%s).",
      interpolation_method.c_str(), to_string(DEFAULT_INTERPOLATION).c_str());
    return DEFAULT_INTERPOLATION;
  }
}

}  // namespace interpolation_methods
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
