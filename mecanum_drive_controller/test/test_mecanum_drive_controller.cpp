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

#include "test_mecanum_drive_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using mecanum_drive_controller::NR_CMD_ITFS;
using mecanum_drive_controller::NR_REF_ITFS;
using mecanum_drive_controller::NR_STATE_ITFS;

class MecanumDriveControllerTest
: public MecanumDriveControllerFixture<TestableMecanumDriveController>
{
};

namespace
{
// Floating-point value comparison threshold
const double EPS = 1e-3;
}  // namespace

// checking if all parameters are initialized and set as expected
TEST_F(MecanumDriveControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.wheels_k, 0.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_TRUE(controller_->params_.joint_names.empty());
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->params_.reference_timeout, 0.1);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.5);
  ASSERT_EQ(controller_->params_.kinematics.wheels_k, 1.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_THAT(controller_->params_.joint_names, testing::ElementsAreArray(joint_names_));
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);

}

// checking if all interfaces, command, state and reference are exported as expected
TEST_F(MecanumDriveControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check command itfs configuration
  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  // check state itfs configuration
  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  // check ref itfs configuration,  reference_names_

  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), NR_REF_ITFS);
  for (size_t i = 0; i < reference_names_.size(); ++i) {
    const std::string ref_itf_name = std::string(controller_->get_node()->get_name()) + "/" +
                                     reference_names_[i] + "/" + interface_name_;
    EXPECT_EQ(reference_interfaces[i].get_name(), ref_itf_name);
    EXPECT_EQ(reference_interfaces[i].get_prefix_name(), controller_->get_node()->get_name());
    EXPECT_EQ(
      reference_interfaces[i].get_interface_name(), reference_names_[i] + "/" + interface_name_);
  }
}

// checking if calling activate() resets the controller reference msg
TEST_F(MecanumDriveControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.readFromNonRT();
  EXPECT_TRUE(std::isnan((*msg)->twist.linear.x));

  ASSERT_TRUE(std::isnan((*msg)->twist.angular.z));

  EXPECT_EQ(controller_->reference_interfaces_.size(), NR_REF_ITFS);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

// checks if return type of update methods are a success
TEST_F(MecanumDriveControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// checks if return type of controller lifecycle methods is a success
TEST_F(MecanumDriveControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

//checks the functionality of on_activate and on_deactivate methods from the controller
TEST_F(MecanumDriveControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_value()));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

//test to check the status of controller state publisher
TEST_F(MecanumDriveControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->reference_interfaces_[0] = 1.5;

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  EXPECT_NEAR(msg.odom.pose.pose.position.y, 0.0, EPS);
  EXPECT_EQ(msg.linear_x_velocity_command, 1.5);


}

// Tests the msg subscriber and publisher 
TEST_F(MecanumDriveControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  joint_command_values_[1] = command_lin_x;

  EXPECT_TRUE(std::isnan(msg.linear_x_velocity_command));

  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
//  w0_vel = 1.0 / params_.kinematics.wheels_radius * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y - params_.kinematics.wheels_k * body_velocity_center_frame_.angular_z);
//  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0) 
  EXPECT_EQ(joint_command_values_[1], 3.0);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.linear_x_velocity_command, 1.5);
}

// Tests controller behavior when too old msg is sent / age_of_last_command > ref_timeout case
TEST_F(MecanumDriveControllerTest, test_sending_too_old_message)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = *(controller_->input_ref_.readFromNonRT());
  auto old_timestamp = reference->header.stamp;
  EXPECT_TRUE(std::isnan(reference->twist.linear.x));
  EXPECT_TRUE(std::isnan(reference->twist.linear.y));
  EXPECT_TRUE(std::isnan(reference->twist.angular.z));

  publish_commands(
    controller_->get_node()->now() - controller_->ref_timeout_ -
    rclcpp::Duration::from_seconds(0.1));
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
  EXPECT_TRUE(std::isnan((reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((reference)->twist.linear.y));
  EXPECT_TRUE(std::isnan((reference)->twist.angular.z));
}

//Tests the case when msg time stamp is zero
TEST_F(MecanumDriveControllerTest, test_time_stamp_zero)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.y));
  EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));

  publish_commands(rclcpp::Time(0));

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp.sec, (*(controller_->input_ref_.readFromNonRT()))->header.stamp.sec);
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 1.5);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y, 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
  EXPECT_NE((*(controller_->input_ref_.readFromNonRT()))->header.stamp.sec, 0.0);
}

//Tests the condition for msg to be accepted, i.e, if age_of_last_command < ref_timeout
TEST_F(MecanumDriveControllerTest, test_message_accepted)
{
  SetUpController();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // try to set command with time before timeout - command is not updated
  auto reference = controller_->input_ref_.readFromNonRT();
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));

  publish_commands(controller_->get_node()->now());

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 1.5);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y, 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

//Test that checks the status of chainable mode and update methods logic
TEST_F(MecanumDriveControllerTest, test_update_logic_not_chainable)
{
  // 1. age>ref_timeout 2. age<ref_timeout
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

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

// Tests behavior of update methods if ref_timeout = 0
TEST_F(MecanumDriveControllerTest, test_ref_timeout_zero_for_update)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();

  // set command statically
  joint_command_values_[1] = command_lin_x;

  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  msg->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.0);
  msg->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg->twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  fprintf(stderr, " age_of_last_command= %f \n", age_of_last_command);
  fprintf(stderr, " controller_->ref_timeout_= %f \n", controller_->ref_timeout_);

  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_FALSE(std::isnan(joint_command_values_[1]));
  EXPECT_NE(joint_command_values_[1], command_lin_x);
//  w0_vel = 1.0 / params_.kinematics.wheels_radius * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y - params_.kinematics.wheels_k * body_velocity_center_frame_.angular_z);
//  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0) 
  EXPECT_EQ(joint_command_values_[1], 3.0);
  ASSERT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->twist.linear.x));
}

// Tests behavior of reference_callback() if ref_timeout = 0
TEST_F(MecanumDriveControllerTest, test_ref_timeout_zero_for_reference_callback)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  //reference_callback() is called implicitly when publish_commands() is called.
  publish_commands(controller_->get_node()->now());

  ASSERT_TRUE(controller_->wait_for_commands(executor));

  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 1.5);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y, 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
