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

#ifndef TEST_VDA5050_SAFETY_STATE_BROADCASTER_HPP_
#define TEST_VDA5050_SAFETY_STATE_BROADCASTER_HPP_

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "control_msgs/msg/vda5050_safety_state.hpp"
#include "vda5050_safety_state_broadcaster/vda5050_safety_state_broadcaster.hpp"

using Vda5050SafetyStateMsg = control_msgs::msg::VDA5050SafetyState;
using testing::IsEmpty;
using testing::SizeIs;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
constexpr auto NODE_FAILURE = controller_interface::CallbackReturn::FAILURE;
}  // namespace

class FriendVDA5050SafetyStateBroadcaster
: public vda5050_safety_state_broadcaster::Vda5050SafetyStateBroadcaster
{
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, init_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, all_parameters_set_configure_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, no_interfaces_set_activate_fail);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, activate_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, deactivate_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, check_exported_interfaces);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, update_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, publish_status_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, update_broadcasted_success);
  FRIEND_TEST(VDA5050SafetyStateBroadcasterTest, publish_nan_voltage);
};

class VDA5050SafetyStateBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}
  static void TearDownTestCase() {}

  void SetUp()
  {
    // initialize controller
    vda5050_safety_state_broadcaster_ = std::make_unique<FriendVDA5050SafetyStateBroadcaster>();
  }
  void TearDown() { vda5050_safety_state_broadcaster_.reset(nullptr); }

  void SetUpVDA5050SafetyStateBroadcaster(
    const std::string controller_name = "test_vda5050_safety_state_broadcaster")
  {
    ASSERT_EQ(
      vda5050_safety_state_broadcaster_->init(
        controller_name, "", 0, "",
        vda5050_safety_state_broadcaster_->define_custom_node_options()),
      controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    state_ifs.emplace_back(fieldViolation1_itf_);
    state_ifs.emplace_back(fieldViolation2_itf_);
    state_ifs.emplace_back(eStopManual1_itf_);
    state_ifs.emplace_back(eStopManual2_itf_);
    state_ifs.emplace_back(eStopRemote1_itf_);
    state_ifs.emplace_back(eStopRemote2_itf_);
    state_ifs.emplace_back(eStopAutoack_itf_);

    vda5050_safety_state_broadcaster_->assign_interfaces({}, std::move(state_ifs));
  }

protected:
  std::array<double, 7> itfs_values_ = {{
    0.0,  // 0 fieldViolation1
    1.0,  // 1 fieldViolation2
    0.0,  // 2 eStopManual1
    0.0,  // 3 eStopManual2
    0.0,  // 4 eStopRemote1
    1.0,  // 5 eStopRemote2
    1.0,  // 6 eStopAutoack
  }};
  hardware_interface::StateInterface::SharedPtr fieldViolation1_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor1", "fieldViolation", &itfs_values_[0]);
  hardware_interface::StateInterface::SharedPtr fieldViolation2_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor2", "fieldViolation", &itfs_values_[1]);
  hardware_interface::StateInterface::SharedPtr eStopManual1_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor1", "eStopManual", &itfs_values_[2]);
  hardware_interface::StateInterface::SharedPtr eStopManual2_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor2", "eStopManual", &itfs_values_[3]);
  hardware_interface::StateInterface::SharedPtr eStopRemote1_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor1", "eStopRemote", &itfs_values_[4]);
  hardware_interface::StateInterface::SharedPtr eStopRemote2_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor2", "eStopRemote", &itfs_values_[5]);
  hardware_interface::StateInterface::SharedPtr eStopAutoack_itf_ =
    std::make_shared<hardware_interface::StateInterface>(
      "PLC_sensor1", "eStopAutoack", &itfs_values_[6]);

  // Test related parameters
  std::unique_ptr<FriendVDA5050SafetyStateBroadcaster> vda5050_safety_state_broadcaster_;

  void subscribe_and_get_messages(Vda5050SafetyStateMsg & vda5050_safety_state_msg)
  {
    // create a new subscriber
    Vda5050SafetyStateMsg::SharedPtr received_vda5050_safety_state_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto vda5050_safety_state_callback = [&](const Vda5050SafetyStateMsg::SharedPtr msg)
    { received_vda5050_safety_state_msg = msg; };
    auto vda5050_safety_state_subscription =
      test_subscription_node.create_subscription<Vda5050SafetyStateMsg>(
        "/test_vda5050_safety_state_broadcaster/vda5050_safety_state", 10,
        vda5050_safety_state_callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      vda5050_safety_state_broadcaster_->update(
        rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      const auto timeout = std::chrono::milliseconds{5};
      const auto until = test_subscription_node.get_clock()->now() + timeout;
      while ((!received_vda5050_safety_state_msg) &&
             test_subscription_node.get_clock()->now() < until)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      // check if message has been received
      if (received_vda5050_safety_state_msg.get())
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";
    ASSERT_TRUE(received_vda5050_safety_state_msg);

    // take message from subscription
    vda5050_safety_state_msg = *received_vda5050_safety_state_msg;
  }
};

#endif  // TEST_VDA5050_SAFETY_STATE_BROADCASTER_HPP_
