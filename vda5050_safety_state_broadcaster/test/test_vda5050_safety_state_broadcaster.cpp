
// Copyright (c) 2025, b-robotized
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

#include "rclcpp/rclcpp.hpp"
#include "test_vda5050_safety_state_broadcaster.hpp"

// Test correct broadcaster initialization
TEST_F(VDA5050SafetyStateBroadcasterTest, init_success) { SetUpVDA5050SafetyStateBroadcaster(); }

// Test that VDA5050SafetyStateBroadcaster parses parameters correctly and sets up state interfaces
TEST_F(VDA5050SafetyStateBroadcasterTest, all_parameters_set_configure_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check interface configuration
  auto cmd_if_conf = vda5050_safety_state_broadcaster_->command_interface_configuration();
  ASSERT_TRUE(cmd_if_conf.names.empty());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = vda5050_safety_state_broadcaster_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_if_conf.names.size(), itfs_values_.size());
}

// Test fails when no defined interfaces
TEST_F(VDA5050SafetyStateBroadcasterTest, no_interfaces_set_activate_fail)
{
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->init(
      "test_vda5050_safety_state_broadcaster", "", 0, "",
      vda5050_safety_state_broadcaster_->define_custom_node_options()),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_FAILURE);
}

// Test all message initial values
TEST_F(VDA5050SafetyStateBroadcasterTest, activate_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check that the message is reset
  auto msg = vda5050_safety_state_broadcaster_->realtime_vda5050_safety_state_publisher_->msg_;
  EXPECT_EQ(msg.e_stop, control_msgs::msg::VDA5050SafetyState::NONE);
  EXPECT_FALSE(msg.field_violation);
}

TEST_F(VDA5050SafetyStateBroadcasterTest, deactivate_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(VDA5050SafetyStateBroadcasterTest, check_exported_interfaces)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_interfaces = vda5050_safety_state_broadcaster_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), static_cast<size_t>(0));

  auto state_interfaces = vda5050_safety_state_broadcaster_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), itfs_values_.size());
  EXPECT_EQ(state_interfaces.names[0], "PLC_sensor1/fieldViolation");
  EXPECT_EQ(state_interfaces.names[1], "PLC_sensor2/fieldViolation");
  EXPECT_EQ(state_interfaces.names[2], "PLC_sensor1/eStopManual");
  EXPECT_EQ(state_interfaces.names[3], "PLC_sensor2/eStopManual");
  EXPECT_EQ(state_interfaces.names[4], "PLC_sensor1/eStopRemote");
  EXPECT_EQ(state_interfaces.names[5], "PLC_sensor2/eStopRemote");
  EXPECT_EQ(state_interfaces.names[6], "PLC_sensor1/eStopAutoack");
}

TEST_F(VDA5050SafetyStateBroadcasterTest, update_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->update(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// Test correct values published for field violation and e-stop
TEST_F(VDA5050SafetyStateBroadcasterTest, publish_status_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->update(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  Vda5050SafetyStateMsg vda5050_safety_state_msg;
  subscribe_and_get_messages(vda5050_safety_state_msg);

  EXPECT_TRUE(vda5050_safety_state_msg.field_violation);
  EXPECT_EQ(vda5050_safety_state_msg.e_stop, control_msgs::msg::VDA5050SafetyState::REMOTE);
}

// Test update logic for field violation and e-stop
TEST_F(VDA5050SafetyStateBroadcasterTest, update_broadcasted_success)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(fieldViolation2_itf_.set_value(0.0));
  ASSERT_TRUE(eStopManual1_itf_.set_value(1.0));

  Vda5050SafetyStateMsg vda5050_safety_state_msg;
  subscribe_and_get_messages(vda5050_safety_state_msg);

  EXPECT_FALSE(vda5050_safety_state_msg.field_violation);
  EXPECT_EQ(vda5050_safety_state_msg.e_stop, control_msgs::msg::VDA5050SafetyState::MANUAL);
}

TEST_F(VDA5050SafetyStateBroadcasterTest, publish_nan_voltage)
{
  SetUpVDA5050SafetyStateBroadcaster();

  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    vda5050_safety_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(fieldViolation2_itf_.set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(eStopRemote2_itf_.set_value(std::numeric_limits<double>::quiet_NaN()));

  Vda5050SafetyStateMsg vda5050_safety_state_msg;
  subscribe_and_get_messages(vda5050_safety_state_msg);

  EXPECT_FALSE(vda5050_safety_state_msg.field_violation);
  EXPECT_EQ(vda5050_safety_state_msg.e_stop, control_msgs::msg::VDA5050SafetyState::AUTO_ACK);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
