// Copyright 2025 Aarav Gupta
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

#include "test_omni_wheel_drive_controller.hpp"
#include <gtest/gtest.h>
#include "controller_interface/controller_interface_base.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using lifecycle_msgs::msg::State;

class OmniWheelDriveControllerTest
: public OmniWheelDriveControllerFixture<TestableOmniWheelDriveController>
{
};

TEST_F(OmniWheelDriveControllerTest, init_fails_without_parameters)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = controller_name_;
  params.robot_description = urdf_;
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = controller_->define_custom_node_options();

  const auto ret =
    controller_->init(params);
  ASSERT_EQ(ret, controller_interface::return_type::ERROR);
}

TEST_F(OmniWheelDriveControllerTest, configure_fails_with_less_than_three_wheels)
{
  ASSERT_EQ(
    InitController({"first_wheel_joint", "second_wheel_joint"}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(OmniWheelDriveControllerTest, when_controller_configured_expect_properly_exported_interfaces)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check command interfaces configuration
  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), wheel_names_.size());
  for (size_t i = 0; i < command_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(command_interfaces.names[i], wheel_names_[i] + "/" + HW_IF_VELOCITY);
  }
  EXPECT_EQ(
    command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // Check state interfaces configuration
  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), wheel_names_.size());
  for (size_t i = 0; i < state_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(state_interfaces.names[i], wheel_names_[i] + "/" + HW_IF_POSITION);
  }
  EXPECT_EQ(state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // Check reference interfaces configuration
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 3);

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

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_test_prefix_false_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_test_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_blank_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is true but frame_prefix is blank so should not be appended to the frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_test_prefix_false_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_test_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the frame
   * id's instead of the namespace*/
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, configure_succeeds_tf_blank_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  nav_msgs::msg::Odometry odometry_message = controller_->odometry_message_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  std::string ns_prefix = test_namespace.erase(0, 1) + "/";
  /* tf_frame_prefix_enable is true but frame_prefix is blank so namespace should be appended to the
   * frame id's */
  ASSERT_EQ(test_odom_frame_id, ns_prefix + odom_id);
  ASSERT_EQ(test_base_frame_id, ns_prefix + base_link_id);
}

TEST_F(OmniWheelDriveControllerTest, activate_fails_without_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(OmniWheelDriveControllerTest, activate_succeeds_with_pos_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);
  // We implicitly test that by default position feedback is required
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assignResourcesPosFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(OmniWheelDriveControllerTest, activate_succeeds_with_vel_resources_assigned)
{
  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0, {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(false))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assignResourcesVelFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(OmniWheelDriveControllerTest, activate_fails_with_wrong_resources_assigned_1)
{
  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0, {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(false))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assignResourcesPosFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(OmniWheelDriveControllerTest, activate_fails_with_wrong_resources_assigned_2)
{
  ASSERT_EQ(
    InitController(
      wheel_names_, 0.0, {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(true))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assignResourcesVelFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_ERROR);
}

// When not in chained mode, we want to test that
// 1. The controller is configurable and all lifecycle functions work properly
// 2. command_interfaces are set to 0.0 when cmd_vel_timeout_ is exceeded and on deactivation
// 3. command_interfaces are set to correct command values the command messages are not timed-out.
// In particular, make sure that the command_interface is not set to NaN right when it starts up.
TEST_F(OmniWheelDriveControllerTest, chainable_controller_unchained_mode)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->is_chainable());
  ASSERT_TRUE(controller_->set_chained_mode(false));
  ASSERT_FALSE(controller_->is_in_chained_mode());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResourcesPosFeedback();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Check that a late command message causes the command interfaces to be set to 0.0
  publish_twist();

  // delay enough time to trigger the timeout (cmd_vel_timeout_ = 0.5s)
  controller_->wait_for_twist(executor);
  std::this_thread::sleep_for(std::chrono::milliseconds(501));

  ASSERT_EQ(
    controller_->update(
      cmd_vel_publisher_node_->get_clock()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // Now check that a timely published command message sets the command interfaces to the correct
  // values
  publish_twist();
  // wait for msg to be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 5.0, 5.0, -15.0};
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i]);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

// When in chained mode, we want to test that
// 1. The controller is configurable and all lifecycle functions work properly
// 2. command_interfaces are set to 0.0 on deactivation
// 3. command_interfaces are set to correct command values (not set to NaN right when it starts up)
TEST_F(OmniWheelDriveControllerTest, chainable_controller_chained_mode)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->is_chainable());
  ASSERT_TRUE(controller_->set_chained_mode(true));
  ASSERT_TRUE(controller_->is_in_chained_mode());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResourcesPosFeedback();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Imitate preceding controllers by setting reference_interfaces_
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 1.0;
  controller_->reference_interfaces_[2] = 1.0;

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 5.0, 5.0, -15.0};
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i]);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

// Make sure that the controller is properly reset when deactivated
// and accepts new commands as expected when it is activated again.
TEST_F(OmniWheelDriveControllerTest, deactivate_then_activate)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->set_chained_mode(false));

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResourcesPosFeedback();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Now check that a timely published command message sets the command interfaces to the correct
  // values
  publish_twist();
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 5.0, 5.0, -15.0};
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i]);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  // (despite calls to update())
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // Activate again
  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface))
      << "Reference interfaces should initially be NaN on activation";
  }
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // A new command should work as expected
  publish_twist();
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i]);
  }

  // Deactivate again and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor.cancel();
}

TEST_F(OmniWheelDriveControllerTest, command_with_zero_timestamp_is_accepted_with_warning)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->set_chained_mode(false));

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResourcesPosFeedback();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // published command message with zero timestamp sets the command interfaces to the correct values
  publish_twist_timestamped(rclcpp::Time(0, 0, RCL_ROS_TIME));
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 5.0, 5.0, -15.0};
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i]);
  }

  // Deactivate and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor.cancel();
}

TEST_F(OmniWheelDriveControllerTest, 3_wheel_test)
{
  ASSERT_EQ(
    InitController({"wheel_1", "wheel_2", "wheel_3"}, 0.0), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  wheels_pos_states_ = {1, 1, 1};
  wheels_vel_states_ = {1, 1, 1};
  wheels_vel_cmds_ = {0.1, 0.2, 0.3};
  assignResourcesPosFeedback({"wheel_1", "wheel_2", "wheel_3"});

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < 3; i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Check that a published command msg sets the command interfaces to the correct values
  publish_twist();
  // wait for msg to be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 8.66025, -8.66025};
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i], 0.0001);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

TEST_F(OmniWheelDriveControllerTest, 3_wheel_rot_test)
{
  ASSERT_EQ(
    InitController({"wheel_1", "wheel_2", "wheel_3"}, 1.0471975512),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  wheels_pos_states_ = {1, 1, 1};
  wheels_vel_states_ = {1, 1, 1};
  wheels_vel_cmds_ = {0.1, 0.2, 0.3};
  assignResourcesPosFeedback({"wheel_1", "wheel_2", "wheel_3"});

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < 3; i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Check that a published command msg sets the command interfaces to the correct values
  publish_twist();
  // wait for msg to be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-1.33975, 5.0, -18.6603};
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i], 0.0001);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

TEST_F(OmniWheelDriveControllerTest, 4_wheel_rot_test)
{
  ASSERT_EQ(
    InitController({"wheel_1", "wheel_2", "wheel_3", "wheel_4"}, 0.7853981634),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  wheels_pos_states_ = {1, 1, 1, 1};
  wheels_vel_states_ = {1, 1, 1, 1};
  wheels_vel_cmds_ = {0.1, 0.2, 0.3, 0.4};
  assignResourcesPosFeedback({"wheel_1", "wheel_2", "wheel_3", "wheel_4"});

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < 4; i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Check that a published command msg sets the command interfaces to the correct values
  publish_twist();
  // wait for msg to be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-5.0, 9.14214, -5.0, -19.1421};
  for (size_t i = 0; i < 4; i++)
  {
    EXPECT_NEAR(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i], 0.0001);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < 4; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < 4; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

TEST_F(OmniWheelDriveControllerTest, 5_wheel_test)
{
  ASSERT_EQ(
    InitController({"wheel_1", "wheel_2", "wheel_3", "wheel_4", "wheel_5"}, 0.0),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  wheels_pos_states_ = {1, 1, 1, 1, 1};
  wheels_vel_states_ = {1, 1, 1, 1, 1};
  wheels_vel_cmds_ = {0.1, 0.2, 0.3, 0.4, 0.5};
  assignResourcesPosFeedback({"wheel_1", "wheel_2", "wheel_3", "wheel_4", "wheel_5"});

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  // But NaNs should not propagate to command interfaces
  for (size_t i = 0; i < 5; i++)
  {
    ASSERT_FALSE(std::isnan(command_itfs_[i]->get_optional().value()));
  }

  // Check that a published command msg sets the command interfaces to the correct values
  publish_twist();
  // wait for msg to be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::vector<double> expected_wheels_vel_cmds = {-15.0, 1.42040, 8.96802, -2.78768, -17.6007};
  for (size_t i = 0; i < 5; i++)
  {
    EXPECT_NEAR(command_itfs_[i]->get_optional().value(), expected_wheels_vel_cmds[i], 0.0001);
  }

  // Now check that the command interfaces are set to 0.0 on deactivation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  for (size_t i = 0; i < 5; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  for (size_t i = 0; i < 5; i++)
  {
    EXPECT_EQ(command_itfs_[i]->get_optional().value(), 0.0);
  }

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
