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

TEST_F(MecanumDriveControllerTest, when_controller_is_configured_expect_all_parameters_set)
{
  SetUpController();

  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis, 0.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_TRUE(controller_->params_.command_joint_names.empty());
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->params_.reference_timeout, 0.1);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.5);
  ASSERT_EQ(controller_->params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis, 1.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_THAT(
    controller_->params_.command_joint_names, testing::ElementsAreArray(command_joint_names_));
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);
}

// when all command, state and reference interfaces are exported then expect them in storage
TEST_F(MecanumDriveControllerTest, when_controller_configured_expect_properly_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check command itfs configuration
  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], command_joint_names_[i] + "/" + interface_name_);
  }

  // check state itfs configuration
  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], command_joint_names_[i] + "/" + interface_name_);
  }

  // check ref itfs configuration,  reference_names_

  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), NR_REF_ITFS);
  for (size_t i = 0; i < reference_names_.size(); ++i)
  {
    const std::string ref_itf_name = std::string(controller_->get_node()->get_name()) + "/" +
                                     reference_names_[i] + "/" + interface_name_;
    EXPECT_EQ(reference_interfaces[i].get_name(), ref_itf_name);
    EXPECT_EQ(reference_interfaces[i].get_prefix_name(), controller_->get_node()->get_name());
    EXPECT_EQ(
      reference_interfaces[i].get_interface_name(), reference_names_[i] + "/" + interface_name_);
  }
}

TEST_F(MecanumDriveControllerTest, when_controller_is_activated_expect_reference_reset)
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

// when calling update methods expect return type are a success
TEST_F(MecanumDriveControllerTest, when_controller_active_and_update_called_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when controller lifecycle methods expect return type is a success
TEST_F(MecanumDriveControllerTest, when_active_controller_is_deactivated_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

// when calling on_activate, on_deactivate and on_activate methods consecutively 
// expect resetting of reference msg, nan values in command_interfaces and 
// resetting of reference msg respectively
TEST_F(MecanumDriveControllerTest, when_controller_is_reactivated_expect_cmd_itfs_not_set_and_update_success)
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
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when controller state published expect state value in storage
TEST_F(MecanumDriveControllerTest, when_update_is_called_expect_status_message)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->reference_interfaces_[0] = 1.5;

  ControllerStateMsg msg;
  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  EXPECT_NEAR(msg.odometry.pose.pose.position.y, 0.0, EPS);
  EXPECT_EQ(msg.reference_velocity.linear.x, 1.5);
}

// when msg subscribed and published expect value in storage
TEST_F(MecanumDriveControllerTest, when_reference_msg_received_expect_updated_commands_and_status_message)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  joint_command_values_[1] = command_lin_x;

  EXPECT_TRUE(std::isnan(msg.reference_velocity.linear.x));

  // reference_callback() is implicitly called when publish_commands() is called
  publish_commands(controller_->get_node()->now());
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  //  w0_vel = 1.0 / params_.kinematics.wheels_radius *
  // (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y
  // - params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis
  // * body_velocity_center_frame_.angular_z);
  //  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[1], 3.0);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.reference_velocity.linear.x, 1.5);
  ASSERT_EQ(msg.back_left_wheel_velocity, 3.0);
}

// when too old msg is sent expect nan values in reference msg
TEST_F(MecanumDriveControllerTest, when_reference_msg_is_too_old_expect_unset_reference)
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

  // reference_callback() is implicitly called when publish_commands() is called
  publish_commands(
    controller_->get_node()->now() - controller_->ref_timeout_ -
    rclcpp::Duration::from_seconds(0.1));
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
  EXPECT_TRUE(std::isnan((reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((reference)->twist.linear.y));
  EXPECT_TRUE(std::isnan((reference)->twist.angular.z));
}

// when time stamp is zero expect that time stamp is set to current time stamp
TEST_F(MecanumDriveControllerTest, when_reference_msg_has_timestamp_zero_expect_reference_set_and_timestamp_set_to_current_time)
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

  // reference_callback() is implicitly called when publish_commands() is called
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

// when age_of_last_command < ref_timeout expect reference msg is accepted and is in rt buffer
TEST_F(MecanumDriveControllerTest, when_message_has_valid_timestamp_expect_reference_set)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));

  // reference_callback() is implicitly called when publish_commands() is called
  publish_commands(controller_->get_node()->now());

  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 1.5);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.y, 0.0);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

// when not in chainable mode and age_of_last_command > reference_timeout expect
// command_interfaces are set to 0.0 and reference_interfaces set to nan
// followed by
// when not in chainable mode and age_of_last_command < reference_timeout expect
// command_interfaces are calculated to non-nan and reference_interfaces set to nan
TEST_F(MecanumDriveControllerTest, when_reference_message_times_out_expect_commands_are_zeroed)
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
  //  w0_vel = 1.0 / params_.kinematics.wheels_radius
  // * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y
  // - params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis
  // * body_velocity_center_frame_.angular_z);
  //  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[1], 3.0);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 3.0);
  }
}

// when in chainable mode and age_of_last_command < reference_timeout expect
// reference_interfaces set by preceding controller and command_interfaces
// are calculated to non-nan values and reference_interfaces are set to nan
TEST_F(MecanumDriveControllerTest, when_controller_in_chainable_mode_expect_receiving_commands_from_reference_interfaces_directly)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  joint_command_values_[1] = command_lin_x;

  controller_->reference_interfaces_[0] = 1.5;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  // age_of_last_command < ref_timeout_
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);

  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[1], command_lin_x);
  //  w0_vel = 1.0 / params_.kinematics.wheels_radius
  // * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y
  // - params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis
  // * body_velocity_center_frame_.angular_z);
  //  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[1], 3.0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_value(), 3.0);
  }
}

// when ref_timeout = 0 expect reference_msg is accepted and command_interfaces
// are calculated to non-nan values and reference_interfaces are set to nan
TEST_F(MecanumDriveControllerTest, when_reference_timeout_is_zero_expect_reference_msg_being_used_only_once)
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
  //  w0_vel = 1.0 / params_.kinematics.wheels_radius
  // * (body_velocity_center_frame_.linear_x - body_velocity_center_frame_.linear_y
  // - params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis
  // * body_velocity_center_frame_.angular_z);
  //  joint_command_values_[1] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[1], 3.0);
  ASSERT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->twist.linear.x));
}

// when ref_timeout = 0 expect reference_callback() writes reference_msg to rt buffer
// from nonrt thread
TEST_F(MecanumDriveControllerTest, when_ref_timeout_zero_for_reference_callback_expect_reference_msg_in_rt_buffer)
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

  // reference_callback() is called implicitly when publish_commands() is called.
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
