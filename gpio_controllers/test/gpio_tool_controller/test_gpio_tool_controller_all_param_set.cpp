// Copyright (c) 2025, b»robotized by Stogl Robotics
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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_io_gripper_controller.hpp"

// Test setting all params and getting success
TEST_F(IOGripperControllerTest, AllParamsSetSuccess)
{
  RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "Setting all parameters");
  SetUpController();
  RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "Setting up controllers");

  setup_parameters();

  RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "Setup parameters successfully");

  // configure success.
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "Setup parameters successfully");
}

// Test not setting the one param and getting failure
TEST_F(IOGripperControllerTest, AllParamNotSetFailure)
{
  SetUpController();

  setup_parameters_fail();

  // configure success. remaining parameters are default
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}
