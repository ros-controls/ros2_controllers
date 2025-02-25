// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_mecanum_drive_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using mecanum_drive_controller::NR_CMD_ITFS;
using mecanum_drive_controller::NR_REF_ITFS;
using mecanum_drive_controller::NR_STATE_ITFS;

class MecanumDriveControllerTest
: public MecanumDriveControllerFixture<TestableMecanumDriveController>
{
};

TEST_F(MecanumDriveControllerTest, when_controller_is_configured_expect_all_parameters_set)
{
  SetUpController();

  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);

  ASSERT_TRUE(controller_->params_.front_left_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_right_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_right_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_left_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_left_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_left_wheel_state_joint_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->params_.reference_timeout, 0.1);

  ASSERT_EQ(
    controller_->params_.front_left_wheel_command_joint_name, TEST_FRONT_LEFT_CMD_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.front_right_wheel_command_joint_name, TEST_FRONT_RIGHT_CMD_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.rear_right_wheel_command_joint_name, TEST_REAR_RIGHT_CMD_JOINT_NAME);
  ASSERT_EQ(controller_->params_.rear_left_wheel_command_joint_name, TEST_REAR_LEFT_CMD_JOINT_NAME);
  ASSERT_THAT(controller_->command_joint_names_, testing::ElementsAreArray(command_joint_names_));

  ASSERT_EQ(
    controller_->params_.front_left_wheel_state_joint_name, TEST_FRONT_LEFT_STATE_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.front_right_wheel_state_joint_name, TEST_FRONT_RIGHT_STATE_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.rear_right_wheel_state_joint_name, TEST_REAR_RIGHT_STATE_JOINT_NAME);
  ASSERT_EQ(controller_->params_.rear_left_wheel_state_joint_name, TEST_REAR_LEFT_STATE_JOINT_NAME);
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(state_joint_names_));
}

// checking if all interfaces, command and state interfaces are exported as expected
TEST_F(MecanumDriveControllerTest, when_controller_configured_expect_properly_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], command_joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
