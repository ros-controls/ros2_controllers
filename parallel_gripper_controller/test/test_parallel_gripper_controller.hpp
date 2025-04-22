// Copyright 2022 ros2_control development team
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

#ifndef TEST_PARALLEL_GRIPPER_CONTROLLER_HPP_
#define TEST_PARALLEL_GRIPPER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "parallel_gripper_controller/parallel_gripper_action_controller.hpp"

namespace
{
// subclassing and friending so we can access member variables
class FriendGripperController : public parallel_gripper_action_controller::GripperActionController
{
  FRIEND_TEST(GripperControllerTest, CommandSuccessTest);
};

class GripperControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController(
    const std::string & controller_name, controller_interface::return_type expected_result);
  void SetUpHandles();

protected:
  std::unique_ptr<FriendGripperController> controller_;

  // dummy joint state values used for tests
  const std::string joint_name_ = "joint1";
  std::vector<double> joint_states_ = {1.1, 2.1};
  std::vector<double> joint_commands_ = {3.1};

  hardware_interface::StateInterface joint_1_pos_state_{
    joint_name_, hardware_interface::HW_IF_POSITION, &joint_states_[0]};
  hardware_interface::StateInterface joint_1_vel_state_{
    joint_name_, hardware_interface::HW_IF_VELOCITY, &joint_states_[1]};
  hardware_interface::CommandInterface joint_1_cmd_{
    joint_name_, hardware_interface::HW_IF_POSITION, &joint_commands_[0]};
};

}  // anonymous namespace

#endif  // TEST_PARALLEL_GRIPPER_CONTROLLER_HPP_
