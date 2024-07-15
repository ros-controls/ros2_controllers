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

#ifndef TEST_FORWARD_COMMAND_CONTROLLER_HPP_
#define TEST_FORWARD_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "forward_command_controller/forward_command_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_POSITION;

// subclassing and friending so we can access member variables
class FriendForwardCommandController : public forward_command_controller::ForwardCommandController
{
  FRIEND_TEST(ForwardCommandControllerTest, JointsParameterNotSet);
  FRIEND_TEST(ForwardCommandControllerTest, InterfaceParameterNotSet);
  FRIEND_TEST(ForwardCommandControllerTest, JointsParameterIsEmpty);
  FRIEND_TEST(ForwardCommandControllerTest, InterfaceParameterEmpty);
  FRIEND_TEST(ForwardCommandControllerTest, ConfigureParamsSuccess);

  FRIEND_TEST(ForwardCommandControllerTest, ActivateWithWrongJointsNamesFails);
  FRIEND_TEST(ForwardCommandControllerTest, ActivateWithWrongInterfaceNameFails);
  FRIEND_TEST(ForwardCommandControllerTest, ActivateSuccess);
  FRIEND_TEST(ForwardCommandControllerTest, CommandSuccessTest);
  FRIEND_TEST(ForwardCommandControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(ForwardCommandControllerTest, NoCommandCheckTest);
  FRIEND_TEST(ForwardCommandControllerTest, CommandCallbackTest);
  FRIEND_TEST(ForwardCommandControllerTest, ActivateDeactivateCommandsResetSuccess);
};

class ForwardCommandControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();
  void SetUpHandles();

protected:
  std::unique_ptr<FriendForwardCommandController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  std::vector<double> joint_commands_ = {1.1, 2.1, 3.1};

  CommandInterface joint_1_pos_cmd_{joint_names_[0], HW_IF_POSITION, &joint_commands_[0]};
  CommandInterface joint_2_pos_cmd_{joint_names_[1], HW_IF_POSITION, &joint_commands_[1]};
  CommandInterface joint_3_pos_cmd_{joint_names_[2], HW_IF_POSITION, &joint_commands_[2]};
};

#endif  // TEST_FORWARD_COMMAND_CONTROLLER_HPP_
