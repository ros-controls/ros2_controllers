// Copyright (c) 2025 Berkan Tali
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

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "generic_steering_controller/kinematic_model_base.hpp"

namespace mock_kinematic_plugin
{

class MockKinematicModel : public kinematic_model::KinematicModelBase
{
public:
  MockKinematicModel() = default;
  ~MockKinematicModel() override = default;

  void update_reference(const geometry_msgs::msg::Twist &, const rclcpp::Time &) override {}

  void configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &,
    const std::vector<std::string> &,
    const std::vector<std::string> &) override {}

  void update_states(const std::unordered_map<std::string, double> &) override {}


  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    double linear, double angular, bool, bool) override
  {
    return std::make_tuple(std::vector<double>{linear, linear},
        std::vector<double>{angular, angular});
  }

  std::unique_ptr<nav_msgs::msg::Odometry> get_odometry_message(const rclcpp::Duration &) override
  {
    return std::make_unique<nav_msgs::msg::Odometry>();
  }

};

}  // namespace mock_kinematic_plugi
n
