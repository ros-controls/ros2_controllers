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

namespace ros2_control_test_assets
{

const auto valid_robot_urdf =
  R"(
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>

  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <link name="link2">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <link name="tool_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="10.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="10.0"/>
  </joint>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="tool_link"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="10.0"/>
  </joint>

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
</robot>
)";

}  // namespace ros2_control_test_assets

#endif  // TEST_ASSET_ROBOT_DESCRIPTION_HPP_
