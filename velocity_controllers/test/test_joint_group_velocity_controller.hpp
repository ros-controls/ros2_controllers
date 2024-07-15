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

#ifndef TEST_JOINT_GROUP_VELOCITY_CONTROLLER_HPP_
#define TEST_JOINT_GROUP_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "velocity_controllers/joint_group_velocity_controller.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_VELOCITY;

// subclassing and friending so we can access member variables
class FriendJointGroupVelocityController : public velocity_controllers::JointGroupVelocityController
{
  FRIEND_TEST(JointGroupVelocityControllerTest, CommandSuccessTest);
  FRIEND_TEST(JointGroupVelocityControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(JointGroupVelocityControllerTest, CommandCallbackTest);
  FRIEND_TEST(JointGroupVelocityControllerTest, StopJointsOnDeactivateTest);
};

class JointGroupVelocityControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();

protected:
  std::unique_ptr<FriendJointGroupVelocityController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  std::vector<double> joint_commands_ = {1.1, 2.1, 3.1};

  CommandInterface joint_1_cmd_{joint_names_[0], HW_IF_VELOCITY, &joint_commands_[0]};
  CommandInterface joint_2_cmd_{joint_names_[1], HW_IF_VELOCITY, &joint_commands_[1]};
  CommandInterface joint_3_cmd_{joint_names_[2], HW_IF_VELOCITY, &joint_commands_[2]};
};

#endif  // TEST_JOINT_GROUP_VELOCITY_CONTROLLER_HPP_
