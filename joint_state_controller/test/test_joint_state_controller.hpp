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

#ifndef TEST_JOINT_STATE_CONTROLLER_HPP_
#define TEST_JOINT_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "joint_state_controller/joint_state_controller.hpp"

#include "test_robot_hardware/test_robot_hardware.hpp"

class JointStateControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpStateController();

public:
  void ConfigureErrorTest();
  void ConfigureSuccessTest();
  void UpdateTest();
  void JointStatePublishTest();
  void DynamicJointStatePublishTest();

protected:
  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot_;
  std::unique_ptr<joint_state_controller::JointStateController> state_controller_;
};

#endif  // TEST_JOINT_STATE_CONTROLLER_HPP_
