// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef MECANUM_DRIVE_CONTROLLER__VALIDATE_MECANUM_DRIVE_CONTROLLER_PARAMETERS_HPP_
#define MECANUM_DRIVE_CONTROLLER__VALIDATE_MECANUM_DRIVE_CONTROLLER_PARAMETERS_HPP_

#include <string>

#include "parameter_traits/parameter_traits.hpp"
#include "rclcpp/parameter.hpp"
#include "tl_expected/expected.hpp"

namespace parameter_traits
{
tl::expected<void, std::string> forbidden_interface_name_prefix(rclcpp::Parameter const & parameter)
{
  auto const & interface_name = parameter.as_string();

  if (interface_name.rfind("blup_", 0) == 0) {
    return tl::make_unexpected("'interface_name' parameter can not start with 'blup_'");
  }

  return {};
}

}  // namespace parameter_traits

#endif  // MECANUM_DRIVE_CONTROLLER__VALIDATE_MECANUM_DRIVE_CONTROLLER_PARAMETERS_HPP_
