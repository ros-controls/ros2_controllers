// Copyright 2023 AIT Austrian Institute of Technology
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

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "joint_trajectory_controller_plugins/trajectory_controller_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

TEST(TestLoadPidController, load_controller)
{
  rclcpp::init(0, nullptr);

  pluginlib::ClassLoader<joint_trajectory_controller_plugins::TrajectoryControllerBase>
    traj_controller_loader(
      "joint_trajectory_controller_plugins",
      "joint_trajectory_controller_plugins::TrajectoryControllerBase");

  std::shared_ptr<joint_trajectory_controller_plugins::TrajectoryControllerBase> traj_contr;

  auto available_classes = traj_controller_loader.getDeclaredClasses();

  std::cout << "available plugins:" << std::endl;
  for (const auto & available_class : available_classes)
  {
    std::cout << "  " << available_class << std::endl;
  }

  std::string controller_type = "joint_trajectory_controller_plugins::PidTrajectoryPlugin";
  ASSERT_TRUE(traj_controller_loader.isClassAvailable(controller_type));
  ASSERT_NO_THROW(traj_contr = traj_controller_loader.createSharedInstance(controller_type));

  rclcpp::shutdown();
}
