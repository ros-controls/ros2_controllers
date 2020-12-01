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
#include <vector>

#include "gmock/gmock.h"

#include "joint_state_controller/joint_state_controller.hpp"

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
  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  std::vector<double> joint_values_ = {1.1, 2.1, 3.1};

  hardware_interface::StateInterface joint_1_pos_state_{joint_names_[0], "position",
    &joint_values_[0]};
  hardware_interface::StateInterface joint_2_pos_state_{joint_names_[1], "position",
    &joint_values_[1]};
  hardware_interface::StateInterface joint_3_pos_state_{joint_names_[2], "position",
    &joint_values_[2]};
  hardware_interface::StateInterface joint_1_vel_state_{joint_names_[0], "velocity",
    &joint_values_[0]};
  hardware_interface::StateInterface joint_2_vel_state_{joint_names_[1], "velocity",
    &joint_values_[1]};
  hardware_interface::StateInterface joint_3_vel_state_{joint_names_[2], "velocity",
    &joint_values_[2]};
  hardware_interface::StateInterface joint_1_eff_state_{joint_names_[0], "effort",
    &joint_values_[0]};
  hardware_interface::StateInterface joint_2_eff_state_{joint_names_[1], "effort",
    &joint_values_[1]};
  hardware_interface::StateInterface joint_3_eff_state_{joint_names_[2], "effort",
    &joint_values_[2]};

  std::unique_ptr<FriendJointStateController> state_controller_;
};

#endif  // TEST_JOINT_STATE_CONTROLLER_HPP_
