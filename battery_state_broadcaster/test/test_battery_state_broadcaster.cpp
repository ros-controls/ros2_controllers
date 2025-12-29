// Copyright (c) 2025, b-robotized Group
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

#include "rclcpp/rclcpp.hpp"
#include "test_battery_state_broadcaster.hpp"

// Test correct broadcaster initialization
TEST_F(BatteryStateBroadcasterTest, init_success) { SetUpBatteryStateBroadcaster(); }

// Test that BatteryStateBroadcaster parses parameters correctly,
// Test that BatteryStateBroadcaster aggregates interfaces and battery properties correctly,
// sets up state interfaces on configure, and computes aggregated counts/sums.
TEST_F(BatteryStateBroadcasterTest, all_parameters_set_configure_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_TRUE(battery_state_broadcaster_->params_.state_joints.empty());

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(
    battery_state_broadcaster_->state_joints_, testing::ElementsAreArray(state_joint_names_));

  auto interface_params = battery_state_broadcaster_->params_.interfaces.state_joints_map;
  auto properties = battery_state_broadcaster_->params_.state_joints_map;
  EXPECT_EQ(interface_params.at("left_wheel").battery_temperature, true);
  EXPECT_EQ(interface_params.at("left_wheel").battery_current, false);
  EXPECT_EQ(interface_params.at("left_wheel").battery_charge, true);
  EXPECT_EQ(interface_params.at("left_wheel").battery_percentage, false);
  EXPECT_EQ(interface_params.at("left_wheel").battery_power_supply_status, true);
  EXPECT_EQ(interface_params.at("left_wheel").battery_power_supply_health, true);
  EXPECT_EQ(interface_params.at("left_wheel").battery_present, false);

  EXPECT_EQ(interface_params.at("right_wheel").battery_temperature, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_current, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_charge, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_percentage, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_power_supply_status, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_power_supply_health, true);
  EXPECT_EQ(interface_params.at("right_wheel").battery_present, false);

  EXPECT_EQ(properties.at("left_wheel").minimum_voltage, 0.0);
  EXPECT_EQ(properties.at("left_wheel").maximum_voltage, 10.0);
  EXPECT_EQ(properties.at("left_wheel").capacity, 12000.0);
  EXPECT_EQ(properties.at("left_wheel").design_capacity, 13000.0);
  EXPECT_EQ(properties.at("left_wheel").power_supply_technology, 3);
  EXPECT_EQ(properties.at("left_wheel").location, "left_slot");
  EXPECT_EQ(properties.at("left_wheel").serial_number, "left_serial_device");

  EXPECT_EQ(properties.at("right_wheel").minimum_voltage, 0.0);
  EXPECT_EQ(properties.at("right_wheel").maximum_voltage, 15.0);
  EXPECT_EQ(properties.at("right_wheel").capacity, 17000.0);
  EXPECT_EQ(properties.at("right_wheel").design_capacity, 18000.0);
  EXPECT_EQ(properties.at("right_wheel").power_supply_technology, 3);
  EXPECT_EQ(properties.at("right_wheel").location, "right_slot");
  EXPECT_EQ(properties.at("right_wheel").serial_number, "right_serial_device");

  // check property aggregation
  EXPECT_EQ(battery_state_broadcaster_->counts_.temperature_cnt, 2.0);
  EXPECT_EQ(battery_state_broadcaster_->counts_.current_cnt, 1.0);
  EXPECT_EQ(
    battery_state_broadcaster_->counts_.percentage_cnt,
    2.0);  // because min and max voltage are valid
  EXPECT_EQ(battery_state_broadcaster_->sums_.capacity_sum, 29000.0);
  EXPECT_EQ(battery_state_broadcaster_->sums_.design_capacity_sum, 31000.0);

  // check interface configuration
  auto cmd_if_conf = battery_state_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = battery_state_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(12lu));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

// check fails when no defined interfaces
TEST_F(BatteryStateBroadcasterTest, no_interfaces_set_activate_fail)
{
  ASSERT_EQ(
    battery_state_broadcaster_->init(
      "test_battery_state_broadcaster", "", 0, "",
      battery_state_broadcaster_->define_custom_node_options()),
    controller_interface::return_type::OK);

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_FAILURE);
}

// check all msgs initial values
// check logic for combined strings for local & serial number and same or none for power supply
TEST_F(BatteryStateBroadcasterTest, activate_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  RawBatteryStatesMsg raw_battery_states_msg =
    battery_state_broadcaster_->raw_battery_states_realtime_publisher_->msg_;
  EXPECT_EQ(raw_battery_states_msg.battery_states.size(), static_cast<size_t>(2));

  // --- Left wheel ---
  const auto & left = raw_battery_states_msg.battery_states[0];
  EXPECT_EQ(left.header.frame_id, "left_wheel");
  EXPECT_TRUE(std::isnan(left.voltage));
  EXPECT_TRUE(std::isnan(left.temperature));
  EXPECT_TRUE(std::isnan(left.current));
  EXPECT_TRUE(std::isnan(left.charge));
  EXPECT_FLOAT_EQ(left.capacity, 12000.0f);
  EXPECT_FLOAT_EQ(left.design_capacity, 13000.0f);
  EXPECT_TRUE(std::isnan(left.percentage));
  EXPECT_EQ(left.power_supply_status, BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
  EXPECT_EQ(left.power_supply_health, BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
  EXPECT_EQ(left.power_supply_technology, 3);
  EXPECT_TRUE(left.present);
  EXPECT_TRUE(left.cell_voltage.empty());
  EXPECT_TRUE(left.cell_temperature.empty());
  EXPECT_EQ(left.location, "left_slot");
  EXPECT_EQ(left.serial_number, "left_serial_device");

  // --- Right wheel ---
  const auto & right = raw_battery_states_msg.battery_states[1];
  EXPECT_EQ(right.header.frame_id, "right_wheel");
  EXPECT_TRUE(std::isnan(right.voltage));
  EXPECT_TRUE(std::isnan(right.temperature));
  EXPECT_TRUE(std::isnan(right.current));
  EXPECT_TRUE(std::isnan(right.charge));
  EXPECT_FLOAT_EQ(right.capacity, 17000.0f);
  EXPECT_FLOAT_EQ(right.design_capacity, 18000.0f);
  EXPECT_TRUE(std::isnan(right.percentage));
  EXPECT_EQ(right.power_supply_status, BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
  EXPECT_EQ(right.power_supply_health, BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
  EXPECT_EQ(right.power_supply_technology, 3);
  EXPECT_TRUE(right.present);
  EXPECT_TRUE(right.cell_voltage.empty());
  EXPECT_TRUE(right.cell_temperature.empty());
  EXPECT_EQ(right.location, "right_slot");
  EXPECT_EQ(right.serial_number, "right_serial_device");

  BatteryStateMsg battery_state_msg =
    battery_state_broadcaster_->battery_state_realtime_publisher_->msg_;

  EXPECT_TRUE(std::isnan(battery_state_msg.voltage));
  EXPECT_TRUE(std::isnan(battery_state_msg.temperature));
  EXPECT_TRUE(std::isnan(battery_state_msg.current));
  EXPECT_TRUE(std::isnan(battery_state_msg.charge));
  EXPECT_TRUE(std::isnan(battery_state_msg.percentage));
  EXPECT_DOUBLE_EQ(battery_state_msg.capacity, 29000.0);
  EXPECT_DOUBLE_EQ(battery_state_msg.design_capacity, 31000.0);
  EXPECT_EQ(battery_state_msg.power_supply_status, BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
  EXPECT_EQ(battery_state_msg.power_supply_health, BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
  EXPECT_EQ(battery_state_msg.power_supply_technology, BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
  EXPECT_EQ(battery_state_msg.location, "left_slot, right_slot, ");
  EXPECT_EQ(battery_state_msg.serial_number, "left_serial_device, right_serial_device, ");
  EXPECT_TRUE(battery_state_msg.present);
  EXPECT_TRUE(battery_state_msg.cell_voltage.empty());
  EXPECT_TRUE(battery_state_msg.cell_temperature.empty());
}

TEST_F(BatteryStateBroadcasterTest, deactivate_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(BatteryStateBroadcasterTest, check_exported_intefaces)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = battery_state_broadcaster_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), static_cast<size_t>(0));

  auto state_interfaces = battery_state_broadcaster_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), itfs_values_.size());
  EXPECT_EQ(state_interfaces.names[0], "left_wheel/battery_voltage");
  EXPECT_EQ(state_interfaces.names[1], "left_wheel/battery_temperature");
  EXPECT_EQ(state_interfaces.names[2], "left_wheel/battery_charge");
  EXPECT_EQ(state_interfaces.names[3], "left_wheel/battery_power_supply_status");
  EXPECT_EQ(state_interfaces.names[4], "left_wheel/battery_power_supply_health");
  EXPECT_EQ(state_interfaces.names[5], "right_wheel/battery_voltage");
  EXPECT_EQ(state_interfaces.names[6], "right_wheel/battery_temperature");
  EXPECT_EQ(state_interfaces.names[7], "right_wheel/battery_current");
  EXPECT_EQ(state_interfaces.names[8], "right_wheel/battery_charge");
  EXPECT_EQ(state_interfaces.names[9], "right_wheel/battery_percentage");
  EXPECT_EQ(state_interfaces.names[10], "right_wheel/battery_power_supply_status");
  EXPECT_EQ(state_interfaces.names[11], "right_wheel/battery_power_supply_health");
}

TEST_F(BatteryStateBroadcasterTest, update_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

// check correct values published
// check correct aggregation for present, percentage, averages, and sums, and higher criticality
TEST_F(BatteryStateBroadcasterTest, publish_status_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  RawBatteryStatesMsg raw_battery_states_msg;
  BatteryStateMsg battery_state_msg;
  subscribe_and_get_messages(raw_battery_states_msg, battery_state_msg);

  ASSERT_EQ(raw_battery_states_msg.battery_states.size(), 2u);

  // Left wheel
  const auto & left = raw_battery_states_msg.battery_states[0];
  EXPECT_EQ(left.header.frame_id, "left_wheel");
  EXPECT_DOUBLE_EQ(left.voltage, 5.0);
  EXPECT_DOUBLE_EQ(left.temperature, 60.0);
  EXPECT_TRUE(std::isnan(left.current));  // disabled in params
  EXPECT_DOUBLE_EQ(left.charge, 6000.0);
  EXPECT_DOUBLE_EQ(left.capacity, 12000.0);
  EXPECT_DOUBLE_EQ(left.design_capacity, 13000.0);
  // percentage calculated (no interface) = (5.0 - 0.0) * 100 / (10.0 - 0.0) = 50
  EXPECT_DOUBLE_EQ(left.percentage, 50.0);
  EXPECT_EQ(left.power_supply_status, 3);  // from itfs_values_[3]
  EXPECT_EQ(left.power_supply_health, 0);  // from itfs_values_[4]
  EXPECT_EQ(left.power_supply_technology, BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
  EXPECT_TRUE(left.present);  // voltage > 0.0
  EXPECT_EQ(left.location, "left_slot");
  EXPECT_EQ(left.serial_number, "left_serial_device");

  // Right wheel
  const auto & right = raw_battery_states_msg.battery_states[1];
  EXPECT_EQ(right.header.frame_id, "right_wheel");
  EXPECT_DOUBLE_EQ(right.voltage, 10.0);
  EXPECT_DOUBLE_EQ(right.temperature, 80.0);
  EXPECT_DOUBLE_EQ(right.current, 2000.0);
  EXPECT_DOUBLE_EQ(right.charge, 5000.0);
  EXPECT_DOUBLE_EQ(right.capacity, 17000.0);
  EXPECT_DOUBLE_EQ(right.design_capacity, 18000.0);
  EXPECT_DOUBLE_EQ(right.percentage, 66.0);  // directly from itfs_values_[9]
  EXPECT_EQ(right.power_supply_status, 2);   // from itfs_values_[10]
  EXPECT_EQ(right.power_supply_health, 4);   // from itfs_values_[11]
  EXPECT_EQ(right.power_supply_technology, BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
  EXPECT_TRUE(right.present);  // voltage > 0.0
  EXPECT_EQ(right.location, "right_slot");
  EXPECT_EQ(right.serial_number, "right_serial_device");

  // Combined battery state message
  EXPECT_EQ(battery_state_msg.header.frame_id, "");
  EXPECT_DOUBLE_EQ(battery_state_msg.voltage, 7.5);              // average of 5 + 10
  EXPECT_DOUBLE_EQ(battery_state_msg.temperature, 70.0);         // average of 60 + 80
  EXPECT_DOUBLE_EQ(battery_state_msg.current, 2000.0);           // only right wheel contributes
  EXPECT_DOUBLE_EQ(battery_state_msg.charge, 11000.0);           // sum of 6000 + 5000
  EXPECT_DOUBLE_EQ(battery_state_msg.capacity, 29000.0);         // sum of 6000 + 5000
  EXPECT_DOUBLE_EQ(battery_state_msg.design_capacity, 31000.0);  // sum of 6000 + 5000
  EXPECT_DOUBLE_EQ(battery_state_msg.percentage, 58.0);          // average of 50 + 66
  EXPECT_EQ(battery_state_msg.power_supply_status, 3);           // max(3, 2)
  EXPECT_EQ(battery_state_msg.power_supply_health, 4);           // max(0, 4)
  EXPECT_EQ(battery_state_msg.power_supply_technology, BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
  EXPECT_TRUE(battery_state_msg.present);  // voltage > 0.0
  EXPECT_EQ(battery_state_msg.location, "left_slot, right_slot, ");
  EXPECT_EQ(battery_state_msg.serial_number, "left_serial_device, right_serial_device, ");
}

TEST_F(BatteryStateBroadcasterTest, update_broadcasted_success)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(left_voltage_itf_->set_value(10.0));

  RawBatteryStatesMsg raw_battery_states_msg;
  BatteryStateMsg battery_state_msg;
  subscribe_and_get_messages(raw_battery_states_msg, battery_state_msg);

  ASSERT_EQ(raw_battery_states_msg.battery_states.size(), 2u);

  // Left wheel
  const auto & left = raw_battery_states_msg.battery_states[0];
  EXPECT_DOUBLE_EQ(left.voltage, 10.0);
  // percentage calculated (no interface) = (10.0 - 0.0) * 100 / (10.0 - 0.0) = 50
  EXPECT_DOUBLE_EQ(left.percentage, 100.0);
  EXPECT_TRUE(left.present);  // voltage > 0.0

  // Right wheel
  const auto & right = raw_battery_states_msg.battery_states[1];
  EXPECT_DOUBLE_EQ(right.voltage, 10.0);
  EXPECT_DOUBLE_EQ(right.percentage, 66.0);  // directly from itfs_values_[9]
  EXPECT_TRUE(right.present);                // voltage > 0.0

  // Combined battery state message
  EXPECT_DOUBLE_EQ(battery_state_msg.voltage, 10.0);     // average of 10 + 10
  EXPECT_DOUBLE_EQ(battery_state_msg.percentage, 83.0);  // average of 100 + 66
  EXPECT_TRUE(battery_state_msg.present);                // voltage > 0.0
}

TEST_F(BatteryStateBroadcasterTest, publish_nan_voltage)
{
  SetUpBatteryStateBroadcaster();

  ASSERT_EQ(battery_state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(battery_state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(left_voltage_itf_->set_value(std::numeric_limits<double>::quiet_NaN()));

  RawBatteryStatesMsg raw_battery_states_msg;
  BatteryStateMsg battery_state_msg;
  subscribe_and_get_messages(raw_battery_states_msg, battery_state_msg);

  ASSERT_EQ(raw_battery_states_msg.battery_states.size(), 2u);

  // Left wheel
  const auto & left = raw_battery_states_msg.battery_states[0];
  EXPECT_TRUE(std::isnan(left.voltage));
  EXPECT_TRUE(std::isnan(left.percentage));
  EXPECT_FALSE(left.present);  // voltage nan

  // Right wheel
  const auto & right = raw_battery_states_msg.battery_states[1];
  EXPECT_DOUBLE_EQ(right.voltage, 10.0);
  EXPECT_DOUBLE_EQ(right.percentage, 66.0);  // directly from itfs_values_[9]
  EXPECT_TRUE(right.present);                // voltage > 0.0

  // Combined battery state message
  EXPECT_TRUE(std::isnan(battery_state_msg.voltage));     // average of nan + 10
  EXPECT_TRUE(std::isnan(battery_state_msg.percentage));  // average of nan + 66
  EXPECT_TRUE(battery_state_msg.present);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
