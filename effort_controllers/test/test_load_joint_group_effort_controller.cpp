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

TEST(TestLoadJointGroupVelocityController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf),
    executor, "test_controller_manager");

<<<<<<< HEAD:effort_controllers/test/test_load_joint_group_effort_controller.cpp
  ASSERT_NE(
    cm.load_controller(
      "test_joint_group_effort_controller", "effort_controllers/JointGroupEffortController"),
    nullptr);
=======
  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/motion_primitives_forward_controller_params.yaml";
  cm.set_parameter({"test_motion_primitives_forward_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_motion_primitives_forward_controller.type",
     "motion_primitives_controllers/MotionPrimitivesForwardController"});
  ASSERT_NE(cm.load_controller("test_motion_primitives_forward_controller"), nullptr);
>>>>>>> f6bcf6c (fix: correct test_load_controller tests for motion_primitives and pid_controller (#2445)):motion_primitives_controllers/test/test_load_motion_primitives_forward_controller.cpp

  rclcpp::shutdown();
}
