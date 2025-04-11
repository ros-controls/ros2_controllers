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
#include "test_steering_controllers_library_ackermann.hpp"

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
    rear_wheels_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_TRACTION_LEFT_WHEEL],
    rear_wheels_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_RIGHT_WHEEL],
    front_wheels_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_LEFT_WHEEL],
    front_wheels_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_state_values_.size());
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_RIGHT_WHEEL],
    controller_->rear_wheels_state_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_LEFT_WHEEL],
    controller_->rear_wheels_state_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_RIGHT_WHEEL],
    controller_->front_wheels_state_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_LEFT_WHEEL],
    controller_->front_wheels_state_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), joint_reference_interfaces_.size());
  for (size_t i = 0; i < joint_reference_interfaces_.size(); ++i)
  {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + joint_reference_interfaces_[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
  }
  EXPECT_EQ(reference_interfaces[0]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(reference_interfaces[1]->get_interface_name(), hardware_interface::HW_IF_POSITION);


}

// Tests controller update_reference_from_subscribers and
// its two cases for position_feedback true/false behavior
// when too old msg is sent i.e age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_both_update_methods_for_ref_timeout)
{
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
  const double TEST_LINEAR_VELOCITY_X = 1.5;
  const float TEST_STEERING_ANGLE = (float)0.575875;

  std::shared_ptr<ControllerAckermannReferenceMsg> msg = std::make_shared<ControllerAckermannReferenceMsg>();

  // adjusting to achieve age_of_last_command > ref_timeout
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->drive.speed = TEST_LINEAR_VELOCITY_X;
  msg->drive.steering_angle = TEST_STEERING_ANGLE;
  msg->drive.steering_angle_velocity = std::numeric_limits<double>::quiet_NaN();
  msg->drive.acceleration = std::numeric_limits<double>::quiet_NaN();
  msg->drive.jerk = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_ackermann_.writeFromNonRT(msg);

  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_ackermann_.readFromNonRT()))->header.stamp;

  // case 1 position_feedback = false
  controller_->params_.position_feedback = false;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromRT()))->drive.speed, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromNonRT()))->drive.speed, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromNonRT()))->drive.steering_angle, TEST_STEERING_ANGLE);

  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_value(), 0.575875, 1e-6);

  // case 2 position_feedback = true
  controller_->params_.position_feedback = true;

  // adjusting to achieve age_of_last_command > ref_timeout
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->drive.speed = TEST_LINEAR_VELOCITY_X;
  msg->drive.steering_angle = TEST_STEERING_ANGLE;
  msg->drive.steering_angle_velocity = std::numeric_limits<double>::quiet_NaN();
  msg->drive.acceleration = std::numeric_limits<double>::quiet_NaN();
  msg->drive.jerk = std::numeric_limits<double>::quiet_NaN();
  controller_->input_ref_ackermann_.writeFromNonRT(msg);

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromRT()))->drive.speed, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[1]));
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromNonRT()))->drive.speed, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ((*(controller_->input_ref_ackermann_.readFromNonRT()))->drive.steering_angle, TEST_STEERING_ANGLE);

  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_value(), 0.575875, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
