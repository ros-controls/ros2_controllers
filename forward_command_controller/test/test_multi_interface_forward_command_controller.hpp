// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Jack Center, Denis Stogl

#ifndef TEST_MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_
#define TEST_MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "forward_command_controller/multi_interface_forward_command_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

// subclassing and friending so we can access member variables
class FriendMultiInterfaceForwardCommandController
: public forward_command_controller::MultiInterfaceForwardCommandController
{
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, JointsParameterNotSet);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, InterfaceParameterNotSet);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, JointsParameterIsEmpty);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, InterfaceParameterEmpty);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, ConfigureParamsSuccess);

  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, ActivateWithWrongJointsNamesFails);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, ActivateWithWrongInterfaceNameFails);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, ActivateSuccess);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, CommandSuccessTest);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, NoCommandCheckTest);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, CommandCallbackTest);
  FRIEND_TEST(MultiInterfaceForwardCommandControllerTest, ActivateDeactivateCommandsResetSuccess);
};

class MultiInterfaceForwardCommandControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController(
    bool set_params_and_activate = false, const std::vector<rclcpp::Parameter> & parameters = {});

protected:
  std::unique_ptr<FriendMultiInterfaceForwardCommandController> controller_;

  // dummy joint state value used for tests
  const std::string joint_name_ = "joint1";

  double pos_cmd_ = 1.1;
  double vel_cmd_ = 2.1;
  double eff_cmd_ = 3.1;

  CommandInterface::SharedPtr joint_1_pos_cmd_ =
    std::make_shared<CommandInterface>(joint_name_, HW_IF_POSITION, &pos_cmd_);
  CommandInterface::SharedPtr joint_1_vel_cmd_ =
    std::make_shared<CommandInterface>(joint_name_, HW_IF_VELOCITY, &vel_cmd_);
  CommandInterface::SharedPtr joint_1_eff_cmd_ =
    std::make_shared<CommandInterface>(joint_name_, HW_IF_EFFORT, &eff_cmd_);
  rclcpp::executors::SingleThreadedExecutor executor;
};

#endif  // TEST_MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_
