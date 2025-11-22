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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "battery_state_broadcaster/battery_state_broadcaster.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/battery_state_array.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using RawBatteryStatesMsg = sensor_msgs::msg::BatteryStateArray;
using sensor_msgs::msg::BatteryState;
using testing::IsEmpty;
using testing::SizeIs;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
constexpr auto NODE_FAILURE = controller_interface::CallbackReturn::FAILURE;
}  // namespace

// subclassing and friending so we can access member variables
class FriendBatteryStateBroadcaster : public battery_state_broadcaster::BatteryStateBroadcaster
{
  FRIEND_TEST(BatteryStateBroadcasterTest, init_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, all_parameters_set_configure_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, no_interfaces_set_activate_fail);
  FRIEND_TEST(BatteryStateBroadcasterTest, activate_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, deactivate_success);
  FRIEND_TEST(BatteryStateBroadcasterTest, check_exported_intefaces);
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
  }
  void TearDown() { battery_state_broadcaster_.reset(nullptr); }

  void SetUpBatteryStateBroadcaster(
    const std::string controller_name = "test_battery_state_broadcaster")
  {
    ASSERT_EQ(
      battery_state_broadcaster_->init(
        controller_name, "", 0, "", battery_state_broadcaster_->define_custom_node_options()),
      controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    state_ifs.emplace_back(left_voltage_itf_);
    state_ifs.emplace_back(left_temperature_itf_);
    state_ifs.emplace_back(left_charge_itf_);
    state_ifs.emplace_back(left_status_itf_);
    state_ifs.emplace_back(left_health_itf_);

    state_ifs.emplace_back(right_voltage_itf_);
    state_ifs.emplace_back(right_temperature_itf_);
    state_ifs.emplace_back(right_current_itf_);
    state_ifs.emplace_back(right_charge_itf_);
    state_ifs.emplace_back(right_percentage_itf_);
    state_ifs.emplace_back(right_status_itf_);
    state_ifs.emplace_back(right_health_itf_);

    battery_state_broadcaster_->assign_interfaces({}, std::move(state_ifs));
  }

protected:
  // Controller-related parameters
  std::vector<std::string> state_joint_names_ = {"left_wheel", "right_wheel"};
  std::array<double, 12> itfs_values_ = {{
    5.0,     // 0 left_voltage
    60.0,    // 1 left_temperature
    6000.0,  // 2 left_charge
    3.0,     // 3 left_status
    0.0,     // 4 left_health
    10.0,    // 5 right_voltage
    80.0,    // 6 right_temperature
    2000.0,  // 7 right_current
    5000.0,  // 8 right_charge
    66.0,    // 9 right_percentage
    2.0,     // 10 right_status
    4.0      // 11 right_health
  }};
  // std::array<double, 13> itfs_values_ = {{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10}};

  hardware_interface::StateInterface left_voltage_itf_{
    "left_wheel", "battery_voltage", &itfs_values_[0]};
  hardware_interface::StateInterface left_temperature_itf_{
    "left_wheel", "battery_temperature", &itfs_values_[1]};
  hardware_interface::StateInterface left_charge_itf_{
    "left_wheel", "battery_charge", &itfs_values_[2]};
  hardware_interface::StateInterface left_status_itf_{
    "left_wheel", "battery_power_supply_status", &itfs_values_[3]};
  hardware_interface::StateInterface left_health_itf_{
    "left_wheel", "battery_power_supply_health", &itfs_values_[4]};
  hardware_interface::StateInterface right_voltage_itf_{
    "right_wheel", "battery_voltage", &itfs_values_[5]};
  hardware_interface::StateInterface right_temperature_itf_{
    "right_wheel", "battery_temperature", &itfs_values_[6]};
  hardware_interface::StateInterface right_current_itf_{
    "right_wheel", "battery_current", &itfs_values_[7]};
  hardware_interface::StateInterface right_charge_itf_{
    "right_wheel", "battery_charge", &itfs_values_[8]};
  hardware_interface::StateInterface right_percentage_itf_{
    "right_wheel", "battery_percentage", &itfs_values_[9]};
  hardware_interface::StateInterface right_status_itf_{
    "right_wheel", "battery_power_supply_status", &itfs_values_[10]};
  hardware_interface::StateInterface right_health_itf_{
    "right_wheel", "battery_power_supply_health", &itfs_values_[11]};

  // Test related parameters
  std::unique_ptr<FriendBatteryStateBroadcaster> battery_state_broadcaster_;

  void subscribe_and_get_messages(
    RawBatteryStatesMsg & raw_battery_states_msg, BatteryStateMsg & battery_state_msg)
  {
    // create a new subscriber
    RawBatteryStatesMsg::SharedPtr received_raw_battery_states_msg;
    BatteryStateMsg::SharedPtr received_battery_state_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto raw_battery_states_callback = [&](const RawBatteryStatesMsg::SharedPtr msg)
    { received_raw_battery_states_msg = msg; };
    auto battery_state_callback = [&](const BatteryStateMsg::SharedPtr msg)
    { received_battery_state_msg = msg; };
    auto raw_battery_states_subscription =
      test_subscription_node.create_subscription<RawBatteryStatesMsg>(
        "/test_battery_state_broadcaster/raw_battery_states", 10, raw_battery_states_callback);
    auto battery_state_subscription = test_subscription_node.create_subscription<BatteryStateMsg>(
      "/test_battery_state_broadcaster/battery_state", 10, battery_state_callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      battery_state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      const auto timeout = std::chrono::milliseconds{5};
      const auto until = test_subscription_node.get_clock()->now() + timeout;
      while ((!received_battery_state_msg || !received_raw_battery_states_msg) &&
             test_subscription_node.get_clock()->now() < until)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      // check if message has been received
      if (received_raw_battery_states_msg.get() && received_battery_state_msg.get())
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";
    ASSERT_TRUE(received_raw_battery_states_msg);
    ASSERT_TRUE(received_battery_state_msg);

    // take message from subscription
    raw_battery_states_msg = *received_raw_battery_states_msg;
    battery_state_msg = *received_battery_state_msg;
  }
};

#endif  // TEST_BATTERY_STATE_BROADCASTER_HPP_
