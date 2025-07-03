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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class FKClient
{
public:
  explicit FKClient(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  std::optional<geometry_msgs::msg::Pose> computeFK(
    const std::vector<std::string> & joint_names, const std::vector<double> & joint_positions,
    const std::string & from_frame, const std::string & to_link);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client_;
};

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__FK_CLIENT_HPP_
