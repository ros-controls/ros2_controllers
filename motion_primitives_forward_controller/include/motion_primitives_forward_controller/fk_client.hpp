// Copyright (c) 2025, b»robotized
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
// Authors: Mathias Fuhrer

#ifndef MOTION_PRIMITIVES_FORWARD_CONTROLLER__FK_CLIENT_HPP_
#define MOTION_PRIMITIVES_FORWARD_CONTROLLER__FK_CLIENT_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class FKClient : public rclcpp::Node
{
public:
  explicit FKClient(const std::string & node_name = "fk_client");

  geometry_msgs::msg::Pose computeFK(
    const std::vector<std::string> & joint_names, const std::vector<double> & joint_positions,
    const std::string & from_frame, const std::string & to_link);

private:
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
};

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__FK_CLIENT_HPP_
