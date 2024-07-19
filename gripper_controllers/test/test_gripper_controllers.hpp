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

#ifndef TEST_GRIPPER_CONTROLLERS_HPP_
#define TEST_GRIPPER_CONTROLLERS_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "gripper_controllers/gripper_action_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::StateInterface;

namespace
{
// subclassing and friending so we can access member variables
template <const char * HardwareInterface>
class FriendGripperController
: public gripper_action_controller::GripperActionController<HardwareInterface>
{
  FRIEND_TEST(GripperControllerTest, CommandSuccessTest);
};

template <typename T>
class GripperControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();
  void SetUpHandles();

protected:
  std::unique_ptr<FriendGripperController<T::value>> controller_;

  // dummy joint state values used for tests
  const std::string joint_name_ = "joint1";
  std::vector<double> joint_states_ = {1.1, 2.1};
  std::vector<double> joint_commands_ = {3.1};

  StateInterface joint_1_pos_state_{joint_name_, HW_IF_POSITION, &joint_states_[0]};
  StateInterface joint_1_vel_state_{joint_name_, HW_IF_VELOCITY, &joint_states_[1]};
  CommandInterface joint_1_cmd_{joint_name_, T::value, &joint_commands_[0]};
};

}  // anonymous namespace

#endif  // TEST_GRIPPER_CONTROLLERS_HPP_
