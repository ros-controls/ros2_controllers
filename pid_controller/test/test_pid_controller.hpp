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
//
// Authors: Daniel Azanov, Dr. Denis
//

#ifndef TEST_PID_CONTROLLER_HPP_
#define TEST_PID_CONTROLLER_HPP_

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
#include "pid_controller/pid_controller.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using ControllerStateMsg = pid_controller::PidController::ControllerStateMsg;
using ControllerCommandMsg = pid_controller::PidController::ControllerReferenceMsg;
using ControllerModeSrvType = pid_controller::PidController::ControllerModeSrvType;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestablePidController : public pid_controller::PidController
{
  FRIEND_TEST(PidControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(PidControllerTest, activate_success);
  FRIEND_TEST(PidControllerTest, reactivate_success);
  FRIEND_TEST(PidControllerTest, test_feedforward_mode_service);
  FRIEND_TEST(PidControllerTest, test_update_logic_feedforward_off);
  FRIEND_TEST(PidControllerTest, test_update_logic_feedforward_on_with_zero_feedforward_gain);
  FRIEND_TEST(PidControllerTest, test_update_logic_chainable_not_use_subscriber_update);
  FRIEND_TEST(PidControllerTest, test_update_logic_angle_wraparound_off);
  FRIEND_TEST(PidControllerTest, test_update_logic_angle_wraparound_on);
  FRIEND_TEST(PidControllerTest, subscribe_and_get_messages_success);
  FRIEND_TEST(PidControllerTest, receive_message_and_publish_updated_status);
  FRIEND_TEST(PidControllerTest, test_update_chained_feedforward_with_gain);
  FRIEND_TEST(PidControllerTest, test_update_chained_feedforward_off_with_gain);
  FRIEND_TEST(PidControllerDualInterfaceTest, test_chained_feedforward_with_gain_dual_interface);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return pid_controller::PidController::on_configure(previous_state);
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return pid_controller::PidController::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerCommandMsg is received.
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
    wait_for_command(executor, timeout);
  }

  void set_reference(const std::vector<double> & target_value)
  {
    std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
    msg->dof_names = params_.dof_names;
    msg->values.resize(msg->dof_names.size(), 0.0);
    for (size_t i = 0; i < msg->dof_names.size(); ++i)
    {
      msg->values[i] = target_value[i];
    }
    msg->values_dot.resize(msg->dof_names.size(), std::numeric_limits<double>::quiet_NaN());
    input_ref_.writeFromNonRT(msg);
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class PidControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    dof_names_ = {"joint1"};
    command_interface_ = "position";
    state_interfaces_ = {"position"};
    dof_state_values_ = {1.1};
    dof_command_values_ = {101.101};
    reference_and_state_dof_names_ = {"joint1state"};

    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandMsg>(
      "/test_pid_controller/reference", rclcpp::SystemDefaultsQoS());

    service_caller_node_ = std::make_shared<rclcpp::Node>("service_caller");
    feedforward_service_client_ = service_caller_node_->create_client<ControllerModeSrvType>(
      "/test_pid_controller/set_feedforward_control");
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_pid_controller")
  {
    ASSERT_EQ(
      controller_->init(controller_name, "", rclcpp::NodeOptions()),
      controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(dof_names_.size());
    command_ifs.reserve(dof_names_.size());

    for (size_t i = 0; i < dof_names_.size(); ++i)
    {
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        dof_names_[i], command_interface_, &dof_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_ifs.reserve(dof_names_.size() * state_interfaces_.size());
    state_itfs_.reserve(dof_names_.size() * state_interfaces_.size());
    size_t index = 0;
    for (const auto & interface : state_interfaces_)
    {
      for (const auto & dof_name : dof_names_)
      {
        state_itfs_.emplace_back(
          hardware_interface::StateInterface(dof_name, interface, &dof_state_values_[index]));
        state_ifs.emplace_back(state_itfs_.back());
        ++index;
      }
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    ControllerStateMsg::SharedPtr received_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr cb_msg) { received_msg = cb_msg; };
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_pid_controller/controller_state", 10, subs_callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
      const auto timeout = std::chrono::milliseconds{1};
      const auto until = test_subscription_node.get_clock()->now() + timeout;
      while (!received_msg && test_subscription_node.get_clock()->now() < until)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      // check if message has been received
      if (received_msg.get())
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";
    ASSERT_TRUE(received_msg);

    // take message from subscription
    msg = *received_msg;
  }

  void publish_commands(
    const std::vector<double> & values = {0.45}, const std::vector<double> & values_dot = {0.0})
  {
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
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerCommandMsg msg;
    msg.dof_names = dof_names_;
    msg.values = values;
    msg.values_dot = values_dot;

    command_publisher_->publish(msg);
  }

  std::shared_ptr<ControllerModeSrvType::Response> call_service(
    const bool feedforward, rclcpp::Executor & executor)
  {
    auto request = std::make_shared<ControllerModeSrvType::Request>();
    request->data = feedforward;

    bool wait_for_service_ret =
      feedforward_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Service is not available!");
    }
    auto result = feedforward_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  std::vector<std::string> dof_names_;
  std::string command_interface_;
  std::vector<std::string> state_interfaces_;
  std::vector<double> dof_state_values_;
  std::vector<double> dof_command_values_;
  std::vector<std::string> reference_and_state_dof_names_;

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestablePidController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerCommandMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr service_caller_node_;
  rclcpp::Client<ControllerModeSrvType>::SharedPtr feedforward_service_client_;
};

#endif  // TEST_PID_CONTROLLER_HPP_
