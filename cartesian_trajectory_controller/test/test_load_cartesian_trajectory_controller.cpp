// Copyright (c) 2026 ros2_control Development Team
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
#include <string>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

// The generator inherits JTC, so on_init validates JTC's required params (joints, command/state
// interfaces) and on_configure loads the kinematics plugin. Both are supplied via a params file.
TEST(TestLoadCartesianTrajectoryController, load_and_configure)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");

  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/cartesian_trajectory_controller_params.yaml";

  cm.set_parameter({"test_cartesian_trajectory_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_cartesian_trajectory_controller.type",
     "cartesian_trajectory_controller/CartesianTrajectoryController"});

  ASSERT_NE(
    cm.load_controller(
      "test_cartesian_trajectory_controller",
      "cartesian_trajectory_controller/CartesianTrajectoryController"),
    nullptr);

  EXPECT_EQ(
    cm.configure_controller("test_cartesian_trajectory_controller"),
    controller_interface::return_type::OK);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
