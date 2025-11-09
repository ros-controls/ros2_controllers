// Copyright (c) 2025, PAL Robotics
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

#ifndef TEST_GENERIC_STATE_BROADCASTER_HPP_
#define TEST_GENERIC_STATE_BROADCASTER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "generic_state_broadcaster/generic_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

// subclassing and friending so we can access member variables
class FriendGenericStateBroadcaster : public generic_state_broadcaster::GenericStateBroadcaster
{
  FRIEND_TEST(GenericStateBroadcasterTest, FailOnEmptyInterfaceListTest);
  FRIEND_TEST(GenericStateBroadcasterTest, ConfigureOnValidInterfaceListTest);
  FRIEND_TEST(GenericStateBroadcasterTest, ReactivateTheControllerWithDifferentInterfacesTest);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithoutJointsParameter);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithoutJointsParameterInvalidURDF);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithoutJointsParameterWithRobotDescription);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithJointsAndNoInterfaces);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithJointsAndInterfaces);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestWithoutInterfacesParameter);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateDeactivateTestTwoJointsOneInterface);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestOneJointTwoInterfaces);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestTwoJointTwoInterfacesAllMissing);
  FRIEND_TEST(GenericStateBroadcasterTest, ActivateTestTwoJointTwoInterfacesOneMissing);
  FRIEND_TEST(GenericStateBroadcasterTest, TestCustomInterfaceWithoutMapping);
  FRIEND_TEST(GenericStateBroadcasterTest, TestCustomInterfaceMapping);
  FRIEND_TEST(GenericStateBroadcasterTest, TestCustomInterfaceMappingUpdate);
  FRIEND_TEST(GenericStateBroadcasterTest, ExtraJointStatePublishTest);
};

class GenericStateBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  controller_interface::return_type SetUpStateBroadcaster(
    const std::vector<std::string> & interfaces = {});

  controller_interface::return_type init_broadcaster_and_set_parameters(
    const std::string & robot_description, const std::vector<std::string> & interfaces);

  void assign_state_interfaces(const std::vector<std::string> & interfaces = {});

  void test_published_joint_state_message(const std::string & topic);

  void test_published_dynamic_joint_state_message(const std::string & topic);

  void activate_and_get_state_message(
    const std::string & topic, control_msgs::msg::ValuesArray & msg);

protected:
  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  const std::vector<std::string> interface_names_ = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  std::string custom_interface_name_ = "measured_position";
  std::vector<double> joint_values_ = {1.1, 2.1, 3.1};
  double custom_joint_value_ = 3.5;

  hardware_interface::StateInterface::SharedPtr joint_1_pos_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[0], interface_names_[0], &joint_values_[0]);
  hardware_interface::StateInterface::SharedPtr joint_2_pos_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[1], interface_names_[0], &joint_values_[1]);
  hardware_interface::StateInterface::SharedPtr joint_3_pos_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[2], interface_names_[0], &joint_values_[2]);
  hardware_interface::StateInterface::SharedPtr joint_1_vel_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[0], interface_names_[1], &joint_values_[0]);
  hardware_interface::StateInterface::SharedPtr joint_2_vel_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[1], interface_names_[1], &joint_values_[1]);
  hardware_interface::StateInterface::SharedPtr joint_3_vel_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[2], interface_names_[1], &joint_values_[2]);
  hardware_interface::StateInterface::SharedPtr joint_1_eff_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[0], interface_names_[2], &joint_values_[0]);
  hardware_interface::StateInterface::SharedPtr joint_2_eff_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[1], interface_names_[2], &joint_values_[1]);
  hardware_interface::StateInterface::SharedPtr joint_3_eff_state_ =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[2], interface_names_[2], &joint_values_[2]);

  hardware_interface::StateInterface::SharedPtr joint_X_custom_state =
    std::make_shared<hardware_interface::StateInterface>(
      joint_names_[0], custom_interface_name_, &custom_joint_value_);

  std::vector<hardware_interface::StateInterface::SharedPtr> test_interfaces_;

  std::unique_ptr<FriendGenericStateBroadcaster> state_broadcaster_;
  std::string frame_id_ = "base_link";
};

#endif  // TEST_GENERIC_STATE_BROADCASTER_HPP_
