// Copyright (c) 2025 Berkan Tali
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

#include "test_generic_steering_controller.hpp"
#include "gtest/gtest.h"

class GenericSteeringControllerTest
  : public GenericSteeringControllerFixture<TestableGenericSteeringController>
{
};

TEST_F(GenericSteeringControllerTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), joint_command_values_.size());
  EXPECT_EQ(
    cmd_if_conf.names[CMD_TRACTION_RIGHT_WHEEL],
    traction_joints_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_TRACTION_LEFT_WHEEL],
    traction_joints_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_WHEEL],
    steering_joints_names_[0] + "/" + steering_interface_name_);


  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_state_values_.size());
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_RIGHT_WHEEL],
    controller_->traction_joints_state_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_LEFT_WHEEL],
    controller_->traction_joints_state_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_WHEEL],
    controller_->steering_joints_state_names_[0] + "/" + steering_interface_name_);

    auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), joint_reference_interfaces_.size());
  for (size_t i = 0; i < joint_reference_interfaces_.size(); ++i) {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + joint_reference_interfaces_[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(reference_interfaces[i]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

TEST_F(GenericSteeringControllerTest, CommandTimeoutSetsZeroVelocity)
{
  SetUpController();
  controller_->get_node()->set_parameter(rclcpp::Parameter("ref_timeout", 0.5));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  auto msg = std::make_shared<ControllerReferenceMsg>();

  auto now = controller_->get_node()->now();
  msg->header.stamp = now - rclcpp::Duration::from_seconds(0.1); // 0.1s ago, within 0.5s timeout
  msg->twist.linear.x = 1.0;
  msg->twist.angular.z = 0.5;

  // Fresh reference (within timeout)
  controller_->reference_callback(msg);
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));
  controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));

  // Timeout case (older than timeout)
  msg->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(1.0);
  msg->twist.linear.x = 1.0;
  msg->twist.angular.z = 0.5;
  controller_->reference_callback(msg);
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));
  controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
