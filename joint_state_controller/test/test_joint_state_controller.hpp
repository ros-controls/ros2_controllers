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

// subclassing and friending so we can access member varibles
class FriendJointStateController : public joint_state_controller::JointStateController
{
  FRIEND_TEST(JointStateControllerTest, ConfigureErrorTest);
  FRIEND_TEST(JointStateControllerTest, ConfigureSuccessTest);
};

class JointStateControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpStateController();

protected:
  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot_;
  std::unique_ptr<FriendJointStateController> state_controller_;
};

#endif  // TEST_JOINT_STATE_CONTROLLER_HPP_
