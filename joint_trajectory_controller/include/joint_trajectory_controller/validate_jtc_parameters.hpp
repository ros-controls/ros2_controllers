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

#ifndef JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "parameter_traits/parameter_traits.hpp"
#include "rclcpp/parameter.hpp"
#include "rsl/algorithm.hpp"
#include "tl_expected/expected.hpp"

namespace parameter_traits
{
tl::expected<void, std::string> command_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Check if command interfaces combination is valid. Valid combinations are:
  // 1. effort
  // 2. velocity
  // 2. position [velocity, [acceleration]]

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
    interface_types.size() > 1 &&
    !rsl::contains<std::vector<std::string>>(interface_types, "position"))
  {
    return tl::make_unexpected(
      "'velocity' command interface can be used either alone or 'position' "
      "interface has to be present");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration") &&
    (!rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
     !rsl::contains<std::vector<std::string>>(interface_types, "position")))
  {
    return tl::make_unexpected(
      "'acceleration' command interface can only be used if 'velocity' and "
      "'position' interfaces are present");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "effort") &&
    interface_types.size() > 1)
  {
    return tl::make_unexpected("'effort' command interface has to be used alone");
  }

  return {};
}

tl::expected<void, std::string> state_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Valid combinations are
  // 1. position [velocity, [acceleration]]

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
    !rsl::contains<std::vector<std::string>>(interface_types, "position"))
  {
    return tl::make_unexpected(
      "'velocity' state interface cannot be used if 'position' interface "
      "is missing.");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration") &&
    (!rsl::contains<std::vector<std::string>>(interface_types, "position") ||
     !rsl::contains<std::vector<std::string>>(interface_types, "velocity")))
  {
    return tl::make_unexpected(
      "'acceleration' state interface cannot be used if 'position' and 'velocity' "
      "interfaces are not present.");
  }

  return {};
}

}  // namespace parameter_traits

#endif  // JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
