// Copyright 2024 FZI Forschungszentrum Informatik
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

TEST(TestLoadPoseBroadcaster, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm{
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager"};

  const std::string test_file_path =
    std::string{TEST_FILES_DIRECTORY} + "/pose_broadcaster_params.yaml";
  cm.set_parameter({"test_pose_broadcaster.params_file", test_file_path});

  cm.set_parameter({"test_pose_broadcaster.type", "pose_broadcaster/PoseBroadcaster"});

  ASSERT_NE(cm.load_controller("test_pose_broadcaster"), nullptr);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
