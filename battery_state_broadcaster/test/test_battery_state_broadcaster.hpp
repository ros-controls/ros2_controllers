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

#ifndef TEST_BATTERY_STATE_BROADCASTER_HPP_
#define TEST_BATTERY_STATE_BROADCASTER_HPP_

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "battery_state_broadcaster/battery_state_broadcaster.hpp"
#include "controller_interface/test_utils.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "control_msgs/msg/battery_state_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using RawBatteryStatesMsg = control_msgs::msg::BatteryStateArray;
using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using controller_interface::deactivate_succeeds;
using sensor_msgs::msg::BatteryState;
using testing::IsEmpty;
using testing::SizeIs;

// subclassing and friending so we can access member variables
class FriendBatteryStateBroadcaster : public battery_state_broadcaster::BatteryStateBroadcaster
{
  // Re-expose base class members that the tests need.
  using BatteryStateBroadcaster::batteries_;
  using BatteryStateBroadcaster::counts_;
  using BatteryStateBroadcaster::params_;
  using BatteryStateBroadcaster::sums_;

  FRIEND_TEST(BatteryStateBroadcasterTest, init_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, all_parameters_set_configure_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, no_interfaces_set_activate_fail);
  FRIEND_TEST(BatteryStateBroadcasterTest, activate_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, activate_success_legacy);
  FRIEND_TEST(BatteryStateBroadcasterTest, deactivate_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, check_exported_interfaces);
  FRIEND_TEST(BatteryStateBroadcasterTest, update_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, publish_status_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, update_broadcasted_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, publish_nan_voltage);
};

class BatteryStateBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}
  static void TearDownTestCase() {}

  void SetUp()
  {
    // initialize controller
    battery_state_broadcaster_ = std::make_unique<FriendBatteryStateBroadcaster>();

    battery0_voltage_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery0", "battery_voltage");
    std::ignore = battery0_voltage_itf_->set_value(itfs_values_[0]);
    battery0_temperature_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery0", "battery_temperature");
    std::ignore = battery0_temperature_itf_->set_value(itfs_values_[1]);
    battery0_charge_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery0", "battery_charge");
    std::ignore = battery0_charge_itf_->set_value(itfs_values_[2]);
    battery0_status_itf_ = std::make_shared<hardware_interface::StateInterface>(
      "battery0", "battery_power_supply_status");
    std::ignore = battery0_status_itf_->set_value(itfs_values_[3]);
    battery0_health_itf_ = std::make_shared<hardware_interface::StateInterface>(
      "battery0", "battery_power_supply_health");
    std::ignore = battery0_health_itf_->set_value(itfs_values_[4]);

    battery1_voltage_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery1", "battery_voltage");
    std::ignore = battery1_voltage_itf_->set_value(itfs_values_[5]);
    battery1_temperature_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery1", "battery_temperature");
    std::ignore = battery1_temperature_itf_->set_value(itfs_values_[6]);
    battery1_current_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery1", "battery_current");
    std::ignore = battery1_current_itf_->set_value(itfs_values_[7]);
    battery1_charge_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery1", "battery_charge");
    std::ignore = battery1_charge_itf_->set_value(itfs_values_[8]);
    battery1_percentage_itf_ =
      std::make_shared<hardware_interface::StateInterface>("battery1", "battery_percentage");
    std::ignore = battery1_percentage_itf_->set_value(itfs_values_[9]);
    battery1_status_itf_ = std::make_shared<hardware_interface::StateInterface>(
      "battery1", "battery_power_supply_status");
    std::ignore = battery1_status_itf_->set_value(itfs_values_[10]);
    battery1_health_itf_ = std::make_shared<hardware_interface::StateInterface>(
      "battery1", "battery_power_supply_health");
    std::ignore = battery1_health_itf_->set_value(itfs_values_[11]);
  }
  void TearDown() { battery_state_broadcaster_.reset(nullptr); }

  void SetUpBatteryStateBroadcaster(
    const std::string controller_name = "test_battery_state_broadcaster")
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = controller_name;
    params.robot_description = "";
    params.update_rate = 0;
    params.node_namespace = "";
    params.node_options = battery_state_broadcaster_->define_custom_node_options();
    ASSERT_EQ(battery_state_broadcaster_->init(params), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    state_ifs.emplace_back(battery0_voltage_itf_);
    state_ifs.emplace_back(battery0_temperature_itf_);
    state_ifs.emplace_back(battery0_charge_itf_);
    state_ifs.emplace_back(battery0_status_itf_);
    state_ifs.emplace_back(battery0_health_itf_);

    state_ifs.emplace_back(battery1_voltage_itf_);
    state_ifs.emplace_back(battery1_temperature_itf_);
    state_ifs.emplace_back(battery1_current_itf_);
    state_ifs.emplace_back(battery1_charge_itf_);
    state_ifs.emplace_back(battery1_percentage_itf_);
    state_ifs.emplace_back(battery1_status_itf_);
    state_ifs.emplace_back(battery1_health_itf_);

    battery_state_broadcaster_->assign_interfaces({}, std::move(state_ifs));
  }

protected:
  // Controller-related parameters
  std::vector<std::string> battery_names_ = {"battery0", "battery1"};
  std::array<double, 12> itfs_values_ = {{
    5.0,     // 0 battery0_voltage
    60.0,    // 1 battery0_temperature
    6000.0,  // 2 battery0_charge
    3.0,     // 3 battery0_status
    0.0,     // 4 battery0_health
    10.0,    // 5 battery1_voltage
    80.0,    // 6 battery1_temperature
    2000.0,  // 7 battery1_current
    5000.0,  // 8 battery1_charge
    66.0,    // 9 battery1_percentage
    2.0,     // 10 battery1_status
    4.0      // 11 battery1_health
  }};

  hardware_interface::StateInterface::SharedPtr battery0_voltage_itf_;
  hardware_interface::StateInterface::SharedPtr battery0_temperature_itf_;
  hardware_interface::StateInterface::SharedPtr battery0_charge_itf_;
  hardware_interface::StateInterface::SharedPtr battery0_status_itf_;
  hardware_interface::StateInterface::SharedPtr battery0_health_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_voltage_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_temperature_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_current_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_charge_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_percentage_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_status_itf_;
  hardware_interface::StateInterface::SharedPtr battery1_health_itf_;

  // Test related parameters
  std::unique_ptr<FriendBatteryStateBroadcaster> battery_state_broadcaster_;

  void subscribe_and_get_messages(
    RawBatteryStatesMsg & raw_battery_states_msg, BatteryStateMsg & battery_state_msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto raw_battery_states_subscription =
      test_subscription_node.create_subscription<RawBatteryStatesMsg>(
        "/test_battery_state_broadcaster/raw_battery_states", 10,
        [](const RawBatteryStatesMsg::SharedPtr) {});
    auto battery_state_subscription = test_subscription_node.create_subscription<BatteryStateMsg>(
      "/test_battery_state_broadcaster/battery_state", 10, [](const BatteryStateMsg::SharedPtr) {});

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    RawBatteryStatesMsg received_raw_battery_states_msg;
    BatteryStateMsg received_battery_state_msg;
    bool has_raw_battery_states_msg = false;
    bool has_battery_state_msg = false;

    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(raw_battery_states_subscription);
    wait_set.add_subscription(battery_state_subscription);

    int max_sub_check_loop_count = 100;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

      if (wait_set.wait(std::chrono::milliseconds(20)).kind() == rclcpp::WaitResultKind::Ready)
      {
        rclcpp::MessageInfo raw_msg_info;
        rclcpp::MessageInfo battery_msg_info;
        while (raw_battery_states_subscription->take(received_raw_battery_states_msg, raw_msg_info))
        {
          has_raw_battery_states_msg = true;
        }
        while (battery_state_subscription->take(received_battery_state_msg, battery_msg_info))
        {
          has_battery_state_msg = true;
        }
      }

      // Check if messages have been received.
      if (has_raw_battery_states_msg && has_battery_state_msg)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";
    ASSERT_TRUE(has_raw_battery_states_msg);
    ASSERT_TRUE(has_battery_state_msg);

    // take message from subscription
    raw_battery_states_msg = received_raw_battery_states_msg;
    battery_state_msg = received_battery_state_msg;
  }
};

#endif  // TEST_BATTERY_STATE_BROADCASTER_HPP_
