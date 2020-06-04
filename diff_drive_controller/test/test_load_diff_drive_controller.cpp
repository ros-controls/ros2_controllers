// Copyright 2020 PAL Robotics S.L.
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

#include <gtest/gtest.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"
#include "rclcpp/utilities.hpp"

TEST(TestLoadDiffDriveController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<test_robot_hardware::TestRobotHardware> robot =
    std::make_shared<test_robot_hardware::TestRobotHardware>();

  robot->init();

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(robot, executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller(
      "test_diff_drive_controller",
      "diff_drive_controller/DiffDriveController"));
}
