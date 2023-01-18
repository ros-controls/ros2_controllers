// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_steering_controllers_library.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

class SteeringControllersLibraryTest
: public SteeringControllersLibraryFixture<TestableSteeringControllersLibrary>
{
};

// Tests controller behavior when too old msg is sent / age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_update_logic_not_chainable)
{
  // 1. age>ref_timeout 2. age<ref_timeout
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  joint_command_values_[1] = command_lin_x;

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg->twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(controller_->reference_interfaces_[1], 0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_FALSE(std::isnan(interface));
  }

  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));

  EXPECT_EQ(joint_command_values_[1], 0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 0.0);
  }

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg_2->twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg_2->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg_2);
  const auto age_of_last_command_2 =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  // age_of_last_command_2 < ref_timeout_
  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[1], command_lin_x);
  //  w0_vel = 1.0 / params_.kinematics.wheels_radius * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y - params_.kinematics.wheels_k * body_velocity_center_frame_.angular_z);
  //  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[1], 3.0);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_FALSE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 3.0);
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
