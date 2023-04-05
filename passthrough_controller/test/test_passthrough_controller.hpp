// Copyright (c) 2023, PAL Robotics
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
/// \author Sai Kishor Kothakota

#ifndef TEST_PASSTHROUGH_CONTROLLER_HPP
#define TEST_PASSTHROUGH_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/handle.hpp"
#include "passthrough_controller/passthrough_controller.hpp"

using hardware_interface::CommandInterface;

// subclassing and friending so we can access member variables
class FriendPassthroughController : public passthrough_controller::PassthroughController
{
  FRIEND_TEST(PassthroughControllerTest, InterfaceParameterNotSet);
  FRIEND_TEST(PassthroughControllerTest, InterfaceParameterEmpty);
  FRIEND_TEST(PassthroughControllerTest, ConfigureParamsSuccess);

  FRIEND_TEST(PassthroughControllerTest, ActivateWithWrongInterfaceNameFails);
  FRIEND_TEST(PassthroughControllerTest, ActivateSuccess);
  FRIEND_TEST(PassthroughControllerTest, CommandSuccessTest);
  FRIEND_TEST(PassthroughControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(PassthroughControllerTest, NoCommandCheckTest);
  FRIEND_TEST(PassthroughControllerTest, CommandCallbackTest);
  FRIEND_TEST(PassthroughControllerTest, ActivateDeactivateCommandsResetSuccess);
};

class PassthroughControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();
  void SetUpHandles();

protected:
  std::unique_ptr<FriendPassthroughController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  std::vector<double> joint_commands_ = {1.1, 2.1, 3.1};

  CommandInterface joint_1_pos_cmd_{joint_names_[0], "interface", &joint_commands_[0]};
  CommandInterface joint_2_pos_cmd_{joint_names_[1], "interface", &joint_commands_[1]};
  CommandInterface joint_3_pos_cmd_{joint_names_[2], "interface", &joint_commands_[2]};
};

#endif  // TEST_PASSTHROUGH_CONTROLLER_HPP
