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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "test_steering_controllers_library.hpp"

class SteeringControllersLibraryTest
: public SteeringControllersLibraryFixture<TestableSteeringControllersLibrary>
{
};

// checking if all interfaces, command, state and reference are exported as expected
TEST_F(SteeringControllersLibraryTest, check_exported_interfaces)
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
    cmd_if_conf.names[CMD_STEER_RIGHT_WHEEL],
    steering_joints_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_LEFT_WHEEL],
    steering_joints_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_state_values_.size());
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_RIGHT_WHEEL],
    controller_->traction_joints_state_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_LEFT_WHEEL],
    controller_->traction_joints_state_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_RIGHT_WHEEL],
    controller_->steering_joints_state_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_LEFT_WHEEL],
    controller_->steering_joints_state_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), joint_reference_interfaces_.size());
  for (size_t i = 0; i < joint_reference_interfaces_.size(); ++i)
  {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + joint_reference_interfaces_[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(reference_interfaces[i]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

// Tests controller update_reference_from_subscribers and
// for position_feedback behavior
// when too old msg is sent i.e age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_position_feedback_ref_timeout)
{
  SetUpController();
  controller_->params_.position_feedback = true;

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
  const double TEST_LINEAR_VELOCITY_X = 1.5;
  const double TEST_LINEAR_VELOCITY_Y = 0.0;
  const double TEST_ANGULAR_VELOCITY_Z = 0.3;

  ControllerReferenceMsg msg;

  msg.header.stamp = controller_->get_node()->now();
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  // age_of_last_command < ref_timeout_
  auto age_of_last_command =
    controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // are the command_itfs updated?
  EXPECT_NEAR(controller_->command_interfaces_[0].get_value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[1].get_value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[2].get_value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_value(), 0.575875, 1e-6);

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  age_of_last_command = controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_optional().value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);
}
// Tests controller update_reference_from_subscribers and
// for position_feedback=false behavior
// when too old msg is sent i.e age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_velocity_feedback_ref_timeout)
{
  SetUpController();
  controller_->params_.position_feedback = false;

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
  const double TEST_LINEAR_VELOCITY_X = 1.5;
  const double TEST_LINEAR_VELOCITY_Y = 0.0;
  const double TEST_ANGULAR_VELOCITY_Z = 0.3;

  ControllerReferenceMsg msg;

  msg.header.stamp = controller_->get_node()->now();
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  auto age_of_last_command =
    controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // age_of_last_command < ref_timeout_
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // are the command_itfs updated?
  EXPECT_NEAR(controller_->command_interfaces_[0].get_value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[1].get_value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[2].get_value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_value(), 0.575875, 1e-6);

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  age_of_last_command = controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_optional().value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
