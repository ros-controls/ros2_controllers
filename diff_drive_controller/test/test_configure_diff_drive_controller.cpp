// Copyright 2020 PAL Robotics SL.
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

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadDiffDriveController, load_configure_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::diffbot_urdf, true, "test_controller_manager");
  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/config/test_diff_drive_controller_limits.yaml";

  cm.set_parameter({"test_diff_drive_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_diff_drive_controller.type", "diff_drive_controller/DiffDriveController"});
  auto ctr = cm.load_controller("test_diff_drive_controller");
  ASSERT_NE(ctr, nullptr);
  ASSERT_EQ(
    ctr->on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
