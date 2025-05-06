// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#ifndef TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
#define TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motion_primitives_forward_controller/motion_primitives_forward_controller.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "industrial_robot_motion_interfaces/msg/motion_primitive.hpp"
#include "motion_primitives_forward_controller/execution_state.hpp"
#include "motion_primitives_forward_controller/motion_type.hpp"
#include "motion_primitives_forward_controller/ready_for_new_primitive.hpp"
#include "std_msgs/msg/int8.hpp"

using ControllerReferenceMsg = industrial_robot_motion_interfaces::msg::MotionPrimitive;
using ControllerStateMsg = std_msgs::msg::Int8;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableMotionPrimitivesForwardController
: public motion_primitives_forward_controller::MotionPrimitivesForwardController
{
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, activate_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, reactivate_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, receive_message_and_publish_updated_status);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return motion_primitives_forward_controller::MotionPrimitivesForwardController::on_configure(
      previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   */
  void wait_for_command(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, timeout);
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class MotionPrimitivesForwardControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_motion_primitives_forward_controller/reference", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(
    const std::string controller_name = "test_motion_primitives_forward_controller")
  {
    ASSERT_EQ(
      controller_->init(controller_name, "", 0, "", controller_->define_custom_node_options()),
      controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(command_values_.size());
    command_ifs.reserve(command_values_.size());

    for (size_t i = 0; i < command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          interface_namespace_, command_interface_names_[i], &command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(state_values_.size());
    state_ifs.reserve(state_values_.size());

    for (size_t i = 0; i < state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          interface_namespace_, state_interface_names_[i], &state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_motion_primitives_forward_controller/state", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands(
    const std::vector<double> & joint_positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
    double velocity = 0.7, double acceleration = 1.0, double move_time = 2.0,
    double blend_radius = 3.0)
  {
    std::cout << "Publishing command message ..." << std::endl;
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
      std::cout << "Found subscriber for topic: " << topic_name << std::endl;
    };

    auto topic_name = command_publisher_->get_topic_name();
    std::cout << "Waiting for subscriber on topic: " << topic_name << std::endl;
    wait_for_topic(topic_name);

    ControllerReferenceMsg msg;

    // TODO(mathias31415): Add other tests for other motion types
    msg.type = MotionType::LINEAR_JOINT;
    msg.joint_positions = joint_positions;
    msg.blend_radius = blend_radius;

    msg.additional_arguments.resize(3);
    msg.additional_arguments[0].argument_name = "velocity";
    msg.additional_arguments[0].argument_value = velocity;
    msg.additional_arguments[1].argument_name = "acceleration";
    msg.additional_arguments[1].argument_value = acceleration;
    msg.additional_arguments[2].argument_name = "move_time";
    msg.additional_arguments[2].argument_value = move_time;

    command_publisher_->publish(msg);
  }

protected:
  // Controller-related parameters
  std::vector<std::string> command_interface_names_ = {
    "motion_type", "q1",           "q2",         "q3",           "q4",
    "q5",          "q6",           "pos_x",      "pos_y",        "pos_z",
    "pos_qx",      "pos_qy",       "pos_qz",     "pos_qw",       "pos_via_x",
    "pos_via_y",   "pos_via_z",    "pos_via_qx", "pos_via_qy",   "pos_via_qz",
    "pos_via_qw",  "blend_radius", "velocity",   "acceleration", "move_time"};

  std::vector<std::string> state_interface_names_ = {"execution_status", "ready_for_new_primitive"};

  std::string interface_namespace_ = "motion_primitive";
  std::array<double, 2> state_values_ = {ExecutionState::IDLE, ReadyForNewPrimitive::READY};
  std::array<double, 25> command_values_ = {
    101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101,
    101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101,
    101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableMotionPrimitivesForwardController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr service_caller_node_;
};

#endif  // TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
