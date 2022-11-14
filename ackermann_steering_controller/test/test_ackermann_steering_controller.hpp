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

#ifndef TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_ACKERMANN_STEERING_CONTROLLER_HPP_
#define TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_ACKERMANN_STEERING_CONTROLLER_HPP_

#include "gmock/gmock.h"

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// TODO(anyone): replace the state and command message types
using ControllerStateMsgOdom = ackermann_steering_controller::AckermannSteeringController::ControllerStateMsgOdom;
using ControllerStateMsgTf = ackermann_steering_controller::AckermannSteeringController::ControllerStateMsgTf;
using ControllerReferenceMsg = ackermann_steering_controller::AckermannSteeringController::ControllerReferenceMsg;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableAckermannSteeringController : public ackermann_steering_controller::AckermannSteeringController
{
  FRIEND_TEST(AckermannSteeringControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(AckermannSteeringControllerTest, check_exported_intefaces);
  FRIEND_TEST(AckermannSteeringControllerTest, activate_success);
  FRIEND_TEST(AckermannSteeringControllerTest, update_success);
  FRIEND_TEST(AckermannSteeringControllerTest, deactivate_success);
  FRIEND_TEST(AckermannSteeringControllerTest, reactivate_success);
  // FRIEND_TEST(AckermannSteeringControllerTest, test_setting_slow_mode_service);
  // FRIEND_TEST(AckermannSteeringControllerTest, test_update_logic_chainable_fast);
  // FRIEND_TEST(AckermannSteeringControllerTest, test_update_logic_chainable_slow);
  FRIEND_TEST(AckermannSteeringControllerTest, publish_status_success);
  FRIEND_TEST(AckermannSteeringControllerTest, receive_message_and_publish_updated_status);
  // FRIEND_TEST(AckermannSteeringControllerTest, test_message_timeout);
  FRIEND_TEST(AckermannSteeringControllerTest, test_message_accepted);
  // FRIEND_TEST(AckermannSteeringControllerTest, test_update_logic);
  FRIEND_TEST(AckermannSteeringControllerTest, test_ref_timeout_zero_for_update);
  FRIEND_TEST(AckermannSteeringControllerTest, test_ref_timeout_zero_for_reference_callback);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = ackermann_steering_controller::AckermannSteeringController::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS) {
      ref_subscriber_wait_set_.add_subscription(ref_subscriber_);
    }
    return ret;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return ackermann_steering_controller::AckermannSteeringController::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerReferenceMsg msg was received, false if timeout.
   */
  bool wait_for_command(
    rclcpp::Executor & executor, rclcpp::WaitSet & subscriber_wait_set,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success) {
      executor.spin_some();
    }
    return success;
  }

  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, ref_subscriber_wait_set_, timeout);
  }

  // TODO(anyone): add implementation of any methods of your controller is needed

private:
  rclcpp::WaitSet ref_subscriber_wait_set_;
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class AckermannSteeringControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_ackermann_steering_controller/reference", rclcpp::SystemDefaultsQoS());

  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_ackermann_steering_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      rear_wheel_name, hardware_interface::HW_IF_VELOCITY, &joint_command_values_[0]));
    command_ifs.emplace_back(command_itfs_.back());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
        front_steer_name, hardware_interface::HW_IF_VELOCITY, &joint_command_values_[1]));
    command_ifs.emplace_back(command_itfs_.back());
    // TODO(anyone): Add other command interfaces, if any

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      rear_wheel_name, hardware_interface::HW_IF_POSITION, &joint_state_values_[0]));
    state_ifs.emplace_back(state_itfs_.back());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      front_steer_name, hardware_interface::HW_IF_POSITION, &joint_state_values_[1]));
    state_ifs.emplace_back(state_itfs_.back());
    // TODO(anyone): Add other state interfaces, if any

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsgOdom & msg_odom)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback_odom = [&](const ControllerStateMsgOdom::SharedPtr) {};
    auto subscription_odom = test_subscription_node.create_subscription<ControllerStateMsgOdom>(
      "/test_ackermann_steering_controller/odometry", 10, subs_callback_odom);

    // call update to publish the test value
    ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
    ASSERT_EQ(
    controller_->update_and_write_commands(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription_odom);
    while (max_sub_check_loop_count--) {
      controller_->update_reference_from_subscribers(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
      controller_->update_and_write_commands(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready) {
        break;
      }
    }

    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_odom_info;
    ASSERT_TRUE(subscription_odom->take(msg_odom, msg_odom_info));
  }

  // TODO(anyone): add/remove arguments as it suites your command message type
  void publish_commands(
    const rclcpp::Time & stamp, const double & twist_linear_x = 0.45,
    const double & twist_angular_z = 0.0)
  {
    auto wait_for_topic = [&](const auto topic_name) {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0) {
        if (wait_count >= 5) {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;

    msg.header.stamp = stamp;
    msg.twist.linear.x  = twist_linear_x;
    msg.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.z = twist_angular_z;

    command_publisher_->publish(msg);
  }


protected:
  // TODO(anyone): adjust the members as needed
  std::string rear_wheel_name = "rear_wheel_joint";
  std::string front_steer_name = "front_steer_joint";

  // Controller-related parameters
  std::array<double, 2> joint_state_values_ = {1.1, 2.2};
  std::array<double, 2> joint_command_values_ = {101.101, 202.202};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  double ref_timeout_ = 0.2;

  // Test related parameters
  std::unique_ptr<TestableAckermannSteeringController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;

};

#endif  // TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_ACKERMANN_STEERING_CONTROLLER_HPP_
