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

#ifndef TEST_JOINT_STATE_BROADCASTER_HPP_
#define TEST_JOINT_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "joint_state_broadcaster/joint_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

// subclassing and friending so we can access member variables
class FriendJointStateBroadcaster : public joint_state_broadcaster::JointStateBroadcaster
{
  FRIEND_TEST(JointStateBroadcasterTest, ConfigureErrorTest);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTest);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestWithoutJointsParameter);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestWithoutInterfacesParameter);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestTwoJointsOneInterface);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestOneJointTwoInterfaces);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestTwoJointTwoInterfacesAllMissing);
  FRIEND_TEST(JointStateBroadcasterTest, ActivateTestTwoJointTwoInterfacesOneMissing);
  FRIEND_TEST(JointStateBroadcasterTest, TestCustomInterfaceWithoutMapping);
  FRIEND_TEST(JointStateBroadcasterTest, TestCustomInterfaceMapping);
  FRIEND_TEST(JointStateBroadcasterTest, TestCustomInterfaceMappingUpdate);
  FRIEND_TEST(JointStateBroadcasterTest, ExtraJointStatePublishTest);
};

class JointStateBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpStateBroadcaster(
    const std::vector<std::string> & joint_names = {},
    const std::vector<std::string> & interfaces = {});

  void init_broadcaster_and_set_parameters(
    const std::vector<std::string> & joint_names = {},
    const std::vector<std::string> & interfaces = {});

  void assign_state_interfaces(
    const std::vector<std::string> & joint_names = {},
    const std::vector<std::string> & interfaces = {});

  void test_published_joint_state_message(const std::string & topic);

  void test_published_dynamic_joint_state_message(const std::string & topic);

protected:
  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  const std::vector<std::string> interface_names_ = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  std::string custom_interface_name_ = "measured_position";
  std::vector<double> joint_values_ = {1.1, 2.1, 3.1};
  double custom_joint_value_ = 3.5;

  hardware_interface::StateInterface joint_1_pos_state_{
    joint_names_[0], interface_names_[0], &joint_values_[0]};
  hardware_interface::StateInterface joint_2_pos_state_{
    joint_names_[1], interface_names_[0], &joint_values_[1]};
  hardware_interface::StateInterface joint_3_pos_state_{
    joint_names_[2], interface_names_[0], &joint_values_[2]};
  hardware_interface::StateInterface joint_1_vel_state_{
    joint_names_[0], interface_names_[1], &joint_values_[0]};
  hardware_interface::StateInterface joint_2_vel_state_{
    joint_names_[1], interface_names_[1], &joint_values_[1]};
  hardware_interface::StateInterface joint_3_vel_state_{
    joint_names_[2], interface_names_[1], &joint_values_[2]};
  hardware_interface::StateInterface joint_1_eff_state_{
    joint_names_[0], interface_names_[2], &joint_values_[0]};
  hardware_interface::StateInterface joint_2_eff_state_{
    joint_names_[1], interface_names_[2], &joint_values_[1]};
  hardware_interface::StateInterface joint_3_eff_state_{
    joint_names_[2], interface_names_[2], &joint_values_[2]};

  hardware_interface::StateInterface joint_X_custom_state{
    joint_names_[0], custom_interface_name_, &custom_joint_value_};

  std::unique_ptr<FriendJointStateBroadcaster> state_broadcaster_;
};

#endif  // TEST_JOINT_STATE_BROADCASTER_HPP_
