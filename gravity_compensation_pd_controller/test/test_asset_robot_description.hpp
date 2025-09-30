// Copyright (c) 2025, University of Salerno, Automatic Control Group
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
// Authors: Davide Risi

#ifndef TEST_ASSET_ROBOT_DESCRIPTION_HPP_
#define TEST_ASSET_ROBOT_DESCRIPTION_HPP_

#include <string>
#include "ros2_control_test_assets/descriptions.hpp"

namespace ros2_control_test_assets
{

const std::string system_hardware_resources =
  R"(
  <ros2_control name="TestSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="mock_sensor_commands">false</param>
      <param name="state_following_offset">0.0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="effort"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <command_interface name="effort"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="joint3">
      <command_interface name="effort"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
  </ros2_control>
)";

const std::string valid_robot_urdf = 
  std::string(urdf_head) + std::string(system_hardware_resources) + std::string(urdf_tail);

}  // namespace ros2_control_test_assets

#endif  // TEST_ASSET_ROBOT_DESCRIPTION_HPP_