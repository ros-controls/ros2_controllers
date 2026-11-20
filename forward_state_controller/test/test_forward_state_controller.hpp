// Copyright 2024 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#ifndef TEST_FORWARD_STATE_CONTROLLER_HPP_
#define TEST_FORWARD_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "forward_state_controller/forward_state_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::HW_IF_POSITION;

// subclassing and friending so we can access member variables
class FriendForwardStateController : public forward_state_controller::ForwardStateController
{
  FRIEND_TEST(ForwardStateControllerTest, StateInterfacesParameterNotSet);
  FRIEND_TEST(ForwardStateControllerTest, ForwardStateMappingMissing);
  FRIEND_TEST(ForwardStateControllerTest, ConfigureParamsSuccess);
  FRIEND_TEST(ForwardStateControllerTest, ActivateSuccess);
  FRIEND_TEST(ForwardStateControllerTest, UpdateForwardsStateToCommand);
  FRIEND_TEST(ForwardStateControllerTest, UpdateForwardsOneStateToMultipleCommands);
};

class ForwardStateControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController(const std::vector<rclcpp::Parameter> & parameters = {});

protected:
  std::unique_ptr<FriendForwardStateController> controller_;

  const std::string state_joint_name_ = "joint1";
  const std::string cmd_joint_name_1_ = "joint2";
  const std::string cmd_joint_name_2_ = "joint3";

  double state_position_value_ = 5.5;
  double cmd_position_value_1_ = 0.0;
  double cmd_position_value_2_ = 0.0;

  StateInterface::SharedPtr joint1_pos_state_ = std::make_shared<StateInterface>(
    state_joint_name_, HW_IF_POSITION, &state_position_value_);

  CommandInterface::SharedPtr joint2_pos_cmd_ =
    std::make_shared<CommandInterface>(cmd_joint_name_1_, HW_IF_POSITION, &cmd_position_value_1_);
  CommandInterface::SharedPtr joint3_pos_cmd_ =
    std::make_shared<CommandInterface>(cmd_joint_name_2_, HW_IF_POSITION, &cmd_position_value_2_);
};

#endif  // TEST_FORWARD_STATE_CONTROLLER_HPP_
