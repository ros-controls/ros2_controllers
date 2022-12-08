// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "test_ackermann_steering_controller.hpp"

using ackermann_steering_controller::NR_CMD_ITFS;
using ackermann_steering_controller::NR_REF_ITFS;
using ackermann_steering_controller::NR_STATE_ITFS;

class AckermannSteeringControllerTest
: public AckermannSteeringControllerFixture<TestableAckermannSteeringController>
{
};

TEST_F(AckermannSteeringControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  // ASSERT_TRUE(controller_->params_.joints.empty());
  // ASSERT_TRUE(controller_->params_.state_joints.empty());
  // ASSERT_TRUE(controller_->params_.interface_name.empty());
  ASSERT_EQ(controller_->params_.reference_timeout, 1);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // ASSERT_THAT(controller_->params_.joints, testing::ElementsAreArray(joint_names_));
  // ASSERT_TRUE(controller_->params_.state_joints.empty());
  // ASSERT_THAT(controller_->state_joints_, testing::ElementsAreArray(joint_names_));
  // ASSERT_EQ(controller_->params_.interface_name, interface_name_);
  ASSERT_EQ(controller_->params_.reference_timeout, 0.2);
}

TEST_F(AckermannSteeringControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  EXPECT_EQ(
    command_intefaces.names[0],
    controller_->params_.rear_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(
    command_intefaces.names[1],
    controller_->params_.front_steer_name + "/" + hardware_interface::HW_IF_POSITION);

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  EXPECT_EQ(
    state_intefaces.names[0],
    controller_->params_.rear_wheel_name + "/" + hardware_interface::HW_IF_POSITION);
  EXPECT_EQ(
    state_intefaces.names[1],
    controller_->params_.front_steer_name + "/" + hardware_interface::HW_IF_POSITION);

  // // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), NR_REF_ITFS);

  const std::string ref_itf_name_0 = std::string(controller_->get_node()->get_name()) + "/" +
                                     "linear" + "/" + hardware_interface::HW_IF_VELOCITY;
  EXPECT_EQ(reference_interfaces[0].get_name(), ref_itf_name_0);
  EXPECT_EQ(reference_interfaces[0].get_prefix_name(), controller_->get_node()->get_name());
  EXPECT_EQ(
    reference_interfaces[0].get_interface_name(),
    std::string("linear") + "/" + hardware_interface::HW_IF_VELOCITY);

  const std::string ref_itf_name_1 = std::string(controller_->get_node()->get_name()) + "/" +
                                     "angular" + "/" + hardware_interface::HW_IF_POSITION;
  EXPECT_EQ(reference_interfaces[1].get_name(), ref_itf_name_1);
  EXPECT_EQ(reference_interfaces[1].get_prefix_name(), controller_->get_node()->get_name());
  EXPECT_EQ(
    reference_interfaces[1].get_interface_name(),
    std::string("angular") + "/" + hardware_interface::HW_IF_POSITION);
}

TEST_F(AckermannSteeringControllerTest, activate_success)
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

TEST_F(AckermannSteeringControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(AckermannSteeringControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AckermannSteeringControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[NR_CMD_ITFS - 2].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 2].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 2].get_value()));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(AckermannSteeringControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->reference_interfaces_[0] = 2.3;
  controller_->reference_interfaces_[1] = 4.4;
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsgOdom msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.pose.pose.position.z, 0);
}

TEST_F(AckermannSteeringControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->reference_interfaces_[0] = 2.3;
  controller_->reference_interfaces_[1] = 4.4;
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ControllerStateMsgOdom msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.pose.pose.position.z, 0);

  if (controller_->params_.use_stamped_vel)
  {
    publish_commands(controller_->get_node()->now());
  }
  else
  {
    publish_commands_unstamped();
  }
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(joint_command_values_[NR_CMD_ITFS - 2], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.pose.pose.position.z, 0);
}

TEST_F(AckermannSteeringControllerTest, test_sending_too_old_message)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // try to set command with time before timeout - command is not updated
  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));
  if (controller_->params_.use_stamped_vel)
  {
    publish_commands(
      controller_->get_node()->now() - controller_->ref_timeout_ -
      rclcpp::Duration::from_seconds(0.1));
    ASSERT_TRUE(controller_->wait_for_commands(executor));
    ASSERT_EQ(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
    EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
    EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));
  }
  else
  {
    publish_commands_unstamped();
    ASSERT_TRUE(controller_->wait_for_commands(executor));
    ASSERT_NE(old_timestamp, (*(controller_->input_ref_.readFromNonRT()))->header.stamp);
    EXPECT_EQ((*reference)->twist.linear.x, 0.45);
    EXPECT_EQ((*reference)->twist.angular.z, 0.0);
  }

  // EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x,
  // 0.45);
  // EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z,
  // 0.0);
}

TEST_F(AckermannSteeringControllerTest, test_time_stamp_zero)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // try to set command with time before timeout - command is not updated
  auto reference = controller_->input_ref_.readFromNonRT();
  auto old_timestamp = (*reference)->header.stamp;
  EXPECT_TRUE(std::isnan((*reference)->twist.linear.x));
  EXPECT_TRUE(std::isnan((*reference)->twist.angular.z));
  if (controller_->params_.use_stamped_vel)
  {
    publish_commands(rclcpp::Time(0));
  }
  else
  {
    publish_commands_unstamped();
  }
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  ASSERT_EQ(old_timestamp.sec, (*(controller_->input_ref_.readFromNonRT()))->header.stamp.sec);
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

TEST_F(AckermannSteeringControllerTest, test_message_accepted)
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
  if (controller_->params_.use_stamped_vel)
  {
    publish_commands(controller_->get_node()->now());
  }
  else
  {
    publish_commands_unstamped();
  }
  ASSERT_TRUE(controller_->wait_for_commands(executor));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

TEST_F(AckermannSteeringControllerTest, test_update_logic_chainable)
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
  static constexpr double TEST_LINEAR_VELOCITY_X = 1.5;
  static constexpr double TEST_ANGULAR_VELOCITY_Z = 1.1;
  joint_command_values_[NR_CMD_ITFS - 2] = 111;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
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
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_CMD_ITFS - 2], 0);
  ASSERT_EQ(controller_->reference_interfaces_[0], 0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    ASSERT_EQ(interface, 0);
  }

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg_2->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
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
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[NR_CMD_ITFS - 2], 111);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_FALSE(std::isnan(interface));
  }
}

TEST_F(AckermannSteeringControllerTest, test_update_logic)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();

  // set command statically
  static constexpr double TEST_LINEAR_VELOCITY_X = 1.5;
  static constexpr double TEST_ANGULAR_VELOCITY_Z = 1.1;
  joint_command_values_[NR_CMD_ITFS - 2] = 111;
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                      rclcpp::Duration::from_seconds(0.1);
  msg->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_CMD_ITFS - 2], 0);
  ASSERT_EQ(controller_->reference_interfaces_[0], 0);

  std::shared_ptr<ControllerReferenceMsg> msg_2 = std::make_shared<ControllerReferenceMsg>();
  msg_2->header.stamp = controller_->get_node()->now();
  msg_2->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg_2->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg_2->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg_2);
  const auto age_of_last_command_2 =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_CMD_ITFS - 2], TEST_LINEAR_VELOCITY_X);
  EXPECT_NE(joint_command_values_[NR_CMD_ITFS - 2], 111);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
}

TEST_F(AckermannSteeringControllerTest, test_ref_timeout_zero_for_update)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.readFromNonRT();

  // set command statically
  static constexpr double TEST_LINEAR_VELOCITY_X = 1.5;
  static constexpr double TEST_ANGULAR_VELOCITY_Z = 1.1;

  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  msg->header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.0);
  msg->twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.writeFromNonRT(msg);
  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;

  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(), controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(joint_command_values_[NR_STATE_ITFS - 2], TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_FALSE(std::isnan((*(controller_->input_ref_.readFromRT()))->twist.linear.x));
}

TEST_F(AckermannSteeringControllerTest, test_ref_timeout_zero_for_reference_callback)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  if (controller_->params_.use_stamped_vel)
  {
    publish_commands(controller_->get_node()->now());
  }
  else
  {
    publish_commands_unstamped();
  }
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 0.45);
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z, 0.0);
}

//test is expected to fail in case of param use_stamped_vel is true

TEST_F(AckermannSteeringControllerTest, test_age_of_last_command_zero_reference_callback_unstamped)
{
  fprintf(
    stderr,
    "-------------------------Expect to fail when use_stamped_vel is true---------------- \n");
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  publish_commands_unstamped();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  const auto age_of_last_command =
    controller_->get_node()->now() - (*(controller_->input_ref_.readFromNonRT()))->header.stamp;
  ASSERT_TRUE(
    age_of_last_command >= rclcpp::Duration::from_seconds(0) &&
    age_of_last_command <= rclcpp::Duration::from_seconds(0.00001));

  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x));
  EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromNonRT()))->twist.angular.z));
  EXPECT_EQ((*(controller_->input_ref_.readFromNonRT()))->twist.linear.x, 0.45);
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
