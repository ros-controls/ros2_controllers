// Copyright (c) 2025, b»robotized
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
// Authors: Mathias Fuhrer

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_motion_primitives_forward_controller.hpp"

class MotionPrimitivesForwardControllerTest
: public MotionPrimitivesForwardControllerFixture<TestableMotionPrimitivesForwardController>
{
};

TEST_F(MotionPrimitivesForwardControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.tf_prefix.empty());

  ASSERT_TRUE(configure_succeeds(controller_));
}

TEST_F(MotionPrimitivesForwardControllerTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), command_values_.size());
  for (size_t i = 0; i < command_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(
      command_interfaces.names[i], interface_namespace_ + "/" + command_interface_names_[i]);
  }

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), state_values_.size());
  for (size_t i = 0; i < state_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(state_interfaces.names[i], interface_namespace_ + "/" + state_interface_names_[i]);
  }
}

TEST_F(MotionPrimitivesForwardControllerTest, activate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  // check that the message queue is reset
  auto & moprim_queue_ = controller_->moprim_queue_;
  ASSERT_TRUE(moprim_queue_.empty());
}

TEST_F(MotionPrimitivesForwardControllerTest, update_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));
}

TEST_F(MotionPrimitivesForwardControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));

  auto val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_TRUE(activate_succeeds(controller_));

  val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, receive_single_action_goal)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  send_single_motion_sequence_goal();  // filling with default values from the function

  // Wait for the command value to be set
  // This is necessary because the action server might take some time to process the goal
  auto start = std::chrono::steady_clock::now();
  while (std::isnan(controller_->command_interfaces_[1].get_optional().value()) &&
         (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT));  // motion type
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0.1);        // q1 - q6
  EXPECT_EQ(controller_->command_interfaces_[2].get_optional().value(), 0.2);
  EXPECT_EQ(controller_->command_interfaces_[3].get_optional().value(), 0.3);
  EXPECT_EQ(controller_->command_interfaces_[4].get_optional().value(), 0.4);
  EXPECT_EQ(controller_->command_interfaces_[5].get_optional().value(), 0.5);
  EXPECT_EQ(controller_->command_interfaces_[6].get_optional().value(), 0.6);
  EXPECT_EQ(controller_->command_interfaces_[21].get_optional().value(), 3.0);  // blend radius
  EXPECT_EQ(controller_->command_interfaces_[22].get_optional().value(), 0.7);  // velocity
  EXPECT_EQ(controller_->command_interfaces_[23].get_optional().value(), 1.0);  // acceleration
  EXPECT_EQ(controller_->command_interfaces_[24].get_optional().value(), 2.0);  // move time
}

TEST_F(MotionPrimitivesForwardControllerTest, resets_unused_command_interfaces_between_primitives)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  auto joint_primitive = make_linear_joint_primitive();
  ASSERT_TRUE(controller_->moprim_queue_.push(joint_primitive));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0.1);
  for (size_t i = 0; i < joint_primitive.joint_positions.size(); ++i)
  {
    EXPECT_EQ(
      controller_->command_interfaces_[i + 1].get_optional().value(),
      joint_primitive.joint_positions[i]);
  }
  for (size_t i = 7; i <= 20; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  EXPECT_EQ(
    controller_->command_interfaces_[21].get_optional().value(), joint_primitive.blend_radius);
  EXPECT_EQ(controller_->command_interfaces_[22].get_optional().value(), 0.7);
  EXPECT_EQ(controller_->command_interfaces_[23].get_optional().value(), 1.0);
  EXPECT_EQ(controller_->command_interfaces_[24].get_optional().value(), 2.0);

  auto cartesian_primitive = make_linear_cartesian_primitive();
  ASSERT_TRUE(controller_->moprim_queue_.push(cartesian_primitive));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (size_t i = 1; i <= 6; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  EXPECT_EQ(controller_->command_interfaces_[7].get_optional().value(), 0.1);
  EXPECT_EQ(controller_->command_interfaces_[8].get_optional().value(), 0.2);
  EXPECT_EQ(controller_->command_interfaces_[9].get_optional().value(), 0.3);
  EXPECT_EQ(controller_->command_interfaces_[10].get_optional().value(), 0.4);
  EXPECT_EQ(controller_->command_interfaces_[11].get_optional().value(), 0.5);
  EXPECT_EQ(controller_->command_interfaces_[12].get_optional().value(), 0.6);
  EXPECT_EQ(controller_->command_interfaces_[13].get_optional().value(), 0.7);
  for (size_t i = 14; i <= 20; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  for (size_t i = 22; i <= 24; ++i)
  {
    expect_command_interface_is_nan(i);
  }

  auto joint_primitive_without_additional_arguments = make_linear_joint_primitive();
  joint_primitive_without_additional_arguments.additional_arguments.clear();
  ASSERT_TRUE(controller_->moprim_queue_.push(joint_primitive_without_additional_arguments));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (size_t i = 0; i < joint_primitive_without_additional_arguments.joint_positions.size(); ++i)
  {
    EXPECT_EQ(
      controller_->command_interfaces_[i + 1].get_optional().value(),
      joint_primitive_without_additional_arguments.joint_positions[i]);
  }
  for (size_t i = 7; i <= 20; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  for (size_t i = 22; i <= 24; ++i)
  {
    expect_command_interface_is_nan(i);
  }
}

TEST_F(
  MotionPrimitivesForwardControllerTest,
  accepts_linear_cartesian_with_joints_when_hardware_solves_kinematics)
{
  SetUpController(true);

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  MotionPrimitive primitive = make_linear_joint_primitive({1.1, 1.2, 1.3, 1.4, 1.5, 1.6});
  primitive.type =
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_CARTESIAN);

  const auto goal_handle = send_motion_sequence_goal({primitive});
  ASSERT_NE(goal_handle, nullptr);

  spin_until_command_interface_is_set(1);

  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_CARTESIAN));
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 1.1);
  EXPECT_EQ(controller_->command_interfaces_[2].get_optional().value(), 1.2);
  EXPECT_EQ(controller_->command_interfaces_[3].get_optional().value(), 1.3);
  EXPECT_EQ(controller_->command_interfaces_[4].get_optional().value(), 1.4);
  EXPECT_EQ(controller_->command_interfaces_[5].get_optional().value(), 1.5);
  EXPECT_EQ(controller_->command_interfaces_[6].get_optional().value(), 1.6);
  for (size_t i = 7; i <= 20; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  EXPECT_EQ(controller_->command_interfaces_[21].get_optional().value(), primitive.blend_radius);
}

TEST_F(
  MotionPrimitivesForwardControllerTest,
  accepts_linear_joint_with_pose_when_hardware_solves_kinematics)
{
  SetUpController(true);

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  auto primitive = make_linear_cartesian_primitive(2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8);
  primitive.type = static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT);

  const auto goal_handle = send_motion_sequence_goal({primitive});
  ASSERT_NE(goal_handle, nullptr);

  spin_until_command_interface_is_set(7);

  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT));
  for (size_t i = 1; i <= 6; ++i)
  {
    expect_command_interface_is_nan(i);
  }
  EXPECT_EQ(controller_->command_interfaces_[7].get_optional().value(), 2.1);
  EXPECT_EQ(controller_->command_interfaces_[8].get_optional().value(), 2.2);
  EXPECT_EQ(controller_->command_interfaces_[9].get_optional().value(), 2.3);
  EXPECT_EQ(controller_->command_interfaces_[10].get_optional().value(), 2.4);
  EXPECT_EQ(controller_->command_interfaces_[11].get_optional().value(), 2.5);
  EXPECT_EQ(controller_->command_interfaces_[12].get_optional().value(), 2.6);
  EXPECT_EQ(controller_->command_interfaces_[13].get_optional().value(), 2.7);
  EXPECT_EQ(controller_->command_interfaces_[21].get_optional().value(), primitive.blend_radius);
}

TEST_F(MotionPrimitivesForwardControllerTest, rejects_kinematic_substitutions_without_kinematics)
{
  SetUpController(false);

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  MotionPrimitive cartesian_with_joints;
  cartesian_with_joints.type =
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_CARTESIAN);
  cartesian_with_joints.joint_positions = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6};
  EXPECT_EQ(send_motion_sequence_goal({cartesian_with_joints}), nullptr);

  auto joint_with_pose = make_linear_cartesian_primitive();
  joint_with_pose.type =
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT);
  EXPECT_EQ(send_motion_sequence_goal({joint_with_pose}), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
