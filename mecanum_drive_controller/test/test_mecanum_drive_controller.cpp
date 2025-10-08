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

TEST_F(MecanumDriveControllerTest, when_controller_is_configured_expect_all_parameters_set)
{
  SetUpController();

  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis, 0.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_TRUE(controller_->params_.front_left_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_right_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_right_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_left_wheel_command_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_left_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_left_wheel_state_joint_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->params_.reference_timeout, 0.9);
  ASSERT_EQ(controller_->params_.kinematics.wheels_radius, 0.5);
  ASSERT_EQ(controller_->params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis, 1.0);

  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.x, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.y, 0.0);
  ASSERT_EQ(controller_->params_.kinematics.base_frame_offset.theta, 0.0);

  ASSERT_EQ(
    controller_->params_.pose_covariance_diagonal,
    std::vector<double>({0.0, 6.0, 12.0, 18.0, 24.0, 30.0}));
  ASSERT_EQ(
    controller_->params_.twist_covariance_diagonal,
    std::vector<double>({0.0, 7.0, 14.0, 21.0, 28.0, 35.0}));

  ASSERT_EQ(
    controller_->params_.front_left_wheel_command_joint_name, TEST_FRONT_LEFT_CMD_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.front_right_wheel_command_joint_name, TEST_FRONT_RIGHT_CMD_JOINT_NAME);
  ASSERT_EQ(
    controller_->params_.rear_right_wheel_command_joint_name, TEST_REAR_RIGHT_CMD_JOINT_NAME);
  ASSERT_EQ(controller_->params_.rear_left_wheel_command_joint_name, TEST_REAR_LEFT_CMD_JOINT_NAME);
  ASSERT_THAT(controller_->command_joint_names_, testing::ElementsAreArray(command_joint_names_));

  // When state joint names are empty they are the same as the command joint names
  ASSERT_TRUE(controller_->params_.front_left_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.front_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_right_wheel_state_joint_name.empty());
  ASSERT_TRUE(controller_->params_.rear_left_wheel_state_joint_name.empty());
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(command_joint_names_));
}

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

  // check ref itfs configuration,

  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), NR_REF_ITFS);

  for (size_t i = 0; i < reference_interface_names.size(); ++i)
  {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + reference_interface_names[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(reference_interfaces[i]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_false_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(false));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);

  std::array<double, 36> pose_covariance = {
    {0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 12.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 24.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 30.0}};

  std::array<double, 36> twist_covariance = {
    {0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 21.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 28.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 35.0}};

  ASSERT_EQ(odometry_message.pose.covariance, pose_covariance);
  ASSERT_EQ(odometry_message.twist.covariance, twist_covariance);
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(true));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the
  frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_blank_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(true));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is true but frame_prefix is blank so should not be appended to the
  frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_false_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(false));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options, test_namespace);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(true));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options, test_namespace);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the
  frame
   * id's instead of the namespace*/
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(MecanumDriveControllerTest, configure_succeeds_tf_blank_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix_enable", rclcpp::ParameterValue(true));
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_mecanum_drive_controller", node_options, test_namespace);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  std::string ns_prefix = test_namespace.erase(0, 1) + "/";
  /* tf_frame_prefix_enable is true but frame_prefix is blank so namespace should be appended to
  the
   * frame id's */
  ASSERT_EQ(test_odom_frame_id, ns_prefix + odom_id);
  ASSERT_EQ(test_base_frame_id, ns_prefix + base_link_id);
}

TEST_F(MecanumDriveControllerTest, when_controller_is_activated_expect_reference_reset)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.get();
  EXPECT_TRUE(std::isnan(msg.twist.linear.x));
  ASSERT_TRUE(std::isnan(msg.twist.angular.z));
}

TEST_F(MecanumDriveControllerTest, when_controller_active_and_update_called_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MecanumDriveControllerTest, when_active_controller_is_deactivated_expect_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(
  MecanumDriveControllerTest,
  when_controller_is_reactivated_expect_cmd_itfs_not_set_and_update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_optional().value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_optional().value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[NR_CMD_ITFS - 4].get_optional().value()));

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// when update is called expect the previously set reference before calling update,
// inside the controller state message
TEST_F(MecanumDriveControllerTest, when_update_is_called_expect_status_message)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  controller_->reference_interfaces_[0] = 1.5;

  ControllerStateMsg msg;
  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  EXPECT_EQ(msg.reference_velocity.linear.x, 1.5);
}

// reference_interfaces and command_interfaces values depend on the reference_msg,
// the below test shows two cases when reference_msg is not received and when it is received.
TEST_F(
  MecanumDriveControllerTest,
  when_reference_msg_received_expect_updated_commands_and_status_message)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  // no reference_msg sent
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ControllerStateMsg msg;
  subscribe_to_controller_status_execute_update_and_get_messages(msg);
  joint_command_values_[controller_->get_rear_left_wheel_index()] = command_lin_x;

  EXPECT_TRUE(std::isnan(msg.reference_velocity.linear.x));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());
  controller_->wait_for_commands(executor);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // REAR LEFT vel =
  // 1.0 / params_.kinematics.wheels_radius *
  // (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
  // params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
  // velocity_in_center_frame_angular_z_);
  //  joint_command_values_[REAR_LEFT] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[controller_->get_rear_left_wheel_index()], 3.0);

  subscribe_to_controller_status_execute_update_and_get_messages(msg);

  ASSERT_EQ(msg.reference_velocity.linear.x, 1.5);
  ASSERT_EQ(msg.back_left_wheel_velocity, 0.1);
  ASSERT_EQ(msg.back_right_wheel_velocity, 0.1);
}

// when too old msg is sent expect reference msg reset
TEST_F(MecanumDriveControllerTest, when_reference_msg_is_too_old_expect_unset_reference)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.get();
  auto old_timestamp = reference.header.stamp;
  EXPECT_TRUE(std::isnan(reference.twist.linear.x));
  EXPECT_TRUE(std::isnan(reference.twist.linear.y));
  EXPECT_TRUE(std::isnan(reference.twist.angular.z));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(
    controller_->get_node()->now() - controller_->ref_timeout_ -
    rclcpp::Duration::from_seconds(0.1));
  controller_->wait_for_commands(executor);
  ASSERT_EQ(old_timestamp, reference.header.stamp);
  EXPECT_TRUE(std::isnan(reference.twist.linear.x));
  EXPECT_TRUE(std::isnan(reference.twist.linear.y));
  EXPECT_TRUE(std::isnan(reference.twist.angular.z));
}

// when time stamp is zero expect that time stamp is set to current time stamp
TEST_F(
  MecanumDriveControllerTest,
  when_reference_msg_has_timestamp_zero_expect_reference_set_and_timestamp_set_to_current_time)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.get();
  auto old_timestamp = reference.header.stamp;
  EXPECT_TRUE(std::isnan(reference.twist.linear.x));
  EXPECT_TRUE(std::isnan(reference.twist.linear.y));
  EXPECT_TRUE(std::isnan(reference.twist.angular.z));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(rclcpp::Time(0));

  controller_->wait_for_commands(executor);
  reference = controller_->input_ref_.get();

  ASSERT_EQ(old_timestamp.sec, reference.header.stamp.sec);
  EXPECT_FALSE(std::isnan(reference.twist.linear.x));
  EXPECT_FALSE(std::isnan(reference.twist.angular.z));
  EXPECT_EQ(reference.twist.linear.x, 1.5);
  EXPECT_EQ(reference.twist.linear.y, 0.0);
  EXPECT_EQ(reference.twist.angular.z, 0.0);
  EXPECT_NE(reference.header.stamp.sec, 0.0);
}

// when the reference_msg has valid timestamp then the timeout check in reference_callback()
// shall pass and reference_msg is not reset
TEST_F(MecanumDriveControllerTest, when_message_has_valid_timestamp_expect_reference_set)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.get();
  EXPECT_TRUE(std::isnan(reference.twist.linear.x));
  EXPECT_TRUE(std::isnan(reference.twist.angular.z));

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());

  controller_->wait_for_commands(executor);
  reference = controller_->input_ref_.get();
  EXPECT_FALSE(std::isnan(reference.twist.linear.x));
  EXPECT_FALSE(std::isnan(reference.twist.angular.z));
  EXPECT_EQ(reference.twist.linear.x, 1.5);
  EXPECT_EQ(reference.twist.linear.y, 0.0);
  EXPECT_EQ(reference.twist.angular.z, 0.0);
}

// when not in chainable mode and ref_msg_timedout expect
// command_interfaces are set to 0.0 and when ref_msg is not timedout expect
// command_interfaces are set to valid command values
TEST_F(
  MecanumDriveControllerTest,
  when_ref_msg_old_expect_cmnd_itfs_set_to_zero_otherwise_to_valid_cmnds)
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
  joint_command_values_[controller_->get_rear_left_wheel_index()] = command_lin_x;

  ControllerReferenceMsg msg, msg_2;

  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  auto reference = controller_->input_ref_.get();
  const auto age_of_last_command = controller_->get_node()->now() - reference.header.stamp;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(reference.twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[0]));

  EXPECT_EQ(joint_command_values_[controller_->get_rear_left_wheel_index()], 0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_optional().value(), 0.0);
  }

  msg_2.header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.01);
  msg_2.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg_2.twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg_2.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg_2.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg_2.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg_2.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg_2);

  reference = controller_->input_ref_.get();
  const auto age_of_last_command_2 = controller_->get_node()->now() - reference.header.stamp;

  // age_of_last_command_2 < ref_timeout_
  ASSERT_TRUE(age_of_last_command_2 <= controller_->ref_timeout_);
  ASSERT_EQ(reference.twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[controller_->get_rear_left_wheel_index()], command_lin_x);
  // BACK Left vel =
  // 1.0 / params_.kinematics.wheels_radius *
  // (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
  // params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
  // velocity_in_center_frame_angular_z_);
  //  joint_command_values_[controller_->get_rear_left_wheel_index()] = 1.0 / 0.5 * (1.5 - 0.0 - 1 *
  //  0.0)
  EXPECT_EQ(joint_command_values_[controller_->get_rear_left_wheel_index()], 3.0);
  ASSERT_EQ(reference.twist.linear.x, TEST_LINEAR_VELOCITY_X);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_optional().value(), 3.0);
  }
}

// when in chained mode the reference_interfaces of chained controller and command_interfaces
// of preceding controller point to same memory location, hence reference_interfaces are not
// exclusively set by the update method of chained controller
TEST_F(
  MecanumDriveControllerTest,
  when_controller_in_chainable_mode_expect_receiving_commands_from_reference_interfaces_directly)
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
  joint_command_values_[controller_->get_rear_left_wheel_index()] = command_lin_x;
  // imitating preceding controllers command_interfaces setting reference_interfaces of chained
  // controller.
  controller_->reference_interfaces_[0] = 3.0;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  // reference_callback() is implicitly called when publish_commands() is called
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NE(joint_command_values_[controller_->get_rear_left_wheel_index()], command_lin_x);

  // REAR LEFT vel =
  // 1.0 / params_.kinematics.wheels_radius *
  // (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
  // params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
  // velocity_in_center_frame_angular_z_);

  //  joint_command_values_[REAR_LEFT] = 1.0 / 0.5 * (3.0 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[controller_->get_rear_left_wheel_index()], 6.0);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  for (size_t i = 0; i < controller_->command_interfaces_.size(); ++i)
  {
    EXPECT_EQ(controller_->command_interfaces_[i].get_optional().value(), 6.0);
  }
}

// when ref_timeout = 0 expect reference_msg is accepted only once and command_interfaces
// are calculated to valid values and reference_interfaces are unset
TEST_F(
  MecanumDriveControllerTest,
  when_reference_timeout_is_zero_expect_reference_msg_being_used_only_once)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // set command statically
  joint_command_values_[controller_->get_rear_left_wheel_index()] = command_lin_x;

  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  ControllerReferenceMsg msg;

  msg.header.stamp = controller_->get_node()->now() - rclcpp::Duration::from_seconds(0.0);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);
  auto reference = controller_->input_ref_.get();

  const auto age_of_last_command = controller_->get_node()->now() - reference.header.stamp;

  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(reference.twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_FALSE(std::isnan(joint_command_values_[1]));
  EXPECT_NE(joint_command_values_[controller_->get_rear_left_wheel_index()], command_lin_x);
  // REAR LEFT vel =
  // 1.0 / params_.kinematics.wheels_radius *
  // (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
  // params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
  // velocity_in_center_frame_angular_z_);
  //  joint_command_values_[REAR_LEFT] = 1.0 / 0.5 * (1.5 - 0.0 - 1 * 0.0)
  EXPECT_EQ(joint_command_values_[controller_->get_rear_left_wheel_index()], 3.0);
  reference = controller_->input_ref_.get();
  ASSERT_TRUE(std::isnan(reference.twist.linear.x));
}

TEST_F(
  MecanumDriveControllerTest,
  when_ref_timeout_zero_for_reference_callback_expect_reference_msg_being_used_only_once)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto reference = controller_->input_ref_.get();
  EXPECT_TRUE(std::isnan(reference.twist.linear.x));
  EXPECT_TRUE(std::isnan(reference.twist.linear.y));
  EXPECT_TRUE(std::isnan(reference.twist.angular.z));
  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  // reference_callback() is called implicitly when publish_commands() is called.
  // reference_msg is published with provided time stamp when publish_commands( time_stamp)
  // is called
  publish_commands(controller_->get_node()->now());

  controller_->wait_for_commands(executor);
  reference = controller_->input_ref_.get();

  EXPECT_FALSE(std::isnan(reference.twist.linear.x));
  EXPECT_FALSE(std::isnan(reference.twist.linear.y));
  EXPECT_FALSE(std::isnan(reference.twist.angular.z));
  EXPECT_EQ(reference.twist.linear.x, 1.5);
  EXPECT_EQ(reference.twist.linear.y, 0.0);
  EXPECT_EQ(reference.twist.angular.z, 0.0);
}

TEST_F(MecanumDriveControllerTest, SideToSideAndRotationOdometryTest)
{
  // Initialize controller
  SetUpController("test_mecanum_drive_controller_with_rotation");

  // Configure
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check on the base frame offset is set by the params
  EXPECT_EQ(controller_->odometry_.getBaseFrameOffset()[0], 1.0);
  EXPECT_EQ(controller_->odometry_.getBaseFrameOffset()[1], 2.0);
  EXPECT_EQ(controller_->odometry_.getBaseFrameOffset()[2], 3.0);

  // Activate in chained mode
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->is_in_chained_mode(), true);

  // Setup reference interfaces for side to side motion
  auto side_to_side_motion = [this](double linear_y)
  {
    controller_->reference_interfaces_[0] = 0;         // linear x
    controller_->reference_interfaces_[1] = linear_y;  // linear y
    controller_->reference_interfaces_[2] = 0;         // angular z
  };

  // Setup reference interfaces for rotation
  auto rotation_motion = [this](double rotation_velocity)
  {
    controller_->reference_interfaces_[0] = 0;                  // linear x
    controller_->reference_interfaces_[1] = 0;                  // linear y
    controller_->reference_interfaces_[2] = rotation_velocity;  // angular z
  };

  const double update_rate = 50.0;  // 50 Hz
  const double dt = 1.0 / update_rate;
  const double test_duration = 1.0;  // 1 second test
  auto current_time = controller_->get_node()->now();

  auto count = 0;
  for (double t = 0; t < test_duration; t += dt)
  {
    switch (count % 4)
    {
      case 0:
        side_to_side_motion(2.0);
        break;
      case 1:
        rotation_motion(-0.5);
        break;
      case 2:
        side_to_side_motion(-2.0);
        break;
      case 3:
        rotation_motion(0.5);
    }

    // Call update method
    ASSERT_EQ(
      controller_->update(current_time, rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);

    current_time += rclcpp::Duration::from_seconds(dt);
    count++;

    // Update the state of the wheels for subsequent loop
    size_t fl_index = controller_->get_front_left_wheel_index();
    size_t fr_index = controller_->get_front_right_wheel_index();
    size_t rl_index = controller_->get_rear_left_wheel_index();
    size_t rr_index = controller_->get_rear_right_wheel_index();
    joint_state_values_[fl_index] =
      controller_->command_interfaces_[fl_index].get_optional().value();
    joint_state_values_[fr_index] =
      controller_->command_interfaces_[fr_index].get_optional().value();
    joint_state_values_[rl_index] =
      controller_->command_interfaces_[rl_index].get_optional().value();
    joint_state_values_[rr_index] =
      controller_->command_interfaces_[rr_index].get_optional().value();
  }

  RCLCPP_INFO(
    controller_->get_node()->get_logger(), "odometry: %f, %f, %f", controller_->odometry_.getX(),
    controller_->odometry_.getY(), controller_->odometry_.getRz());

  // Verify odometry remains bounded
  EXPECT_LT(std::abs(controller_->odometry_.getX()), 1.0);
  EXPECT_LT(std::abs(controller_->odometry_.getY()), 1.0);
  EXPECT_LT(std::abs(controller_->odometry_.getRz()), M_PI);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
