// Copyright 2022 VoodooIT, sole proprietorship
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

#ifndef TEST_TWIST_CONTROLLER_HPP_
#define TEST_TWIST_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cartesian_controllers/twist_controller.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using ControllerCommandMsg = geometry_msgs::msg::TwistStamped;

namespace
{
constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}

}  // namespace

// subclassing and friending so we can access member variables
class TestableTwistController : public cartesian_controllers::TwistController
{
  FRIEND_TEST(TwistControllerTest, joint_names_parameter_not_set);
  FRIEND_TEST(TwistControllerTest, interface_parameter_not_set);
  FRIEND_TEST(TwistControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(TwistControllerTest, check_intefaces);
  FRIEND_TEST(TwistControllerTest, activate_success);
  FRIEND_TEST(TwistControllerTest, update_success);
  FRIEND_TEST(TwistControllerTest, deactivate_success);
  FRIEND_TEST(TwistControllerTest, reactivate_success);
  FRIEND_TEST(TwistControllerTest, command_callback_test);

public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = cartesian_controllers::TwistController::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS)
    {
      command_subscriber_wait_set_.add_subscription(twist_command_subscriber_);
    }
    return ret;
  }

  /**
   * @brief wait_for_command blocks until a new ControllerCommandMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerCommandMsg msg was received, false if timeout.
   */
  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success =
      command_subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

private:
  rclcpp::WaitSet command_subscriber_wait_set_;
};

class TwistControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<TestableTwistController>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandMsg>(
      "/test_twist_controller/commands", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(bool set_parameters = true)
  {
    const auto result = controller_->init("test_twist_controller");
    ASSERT_EQ(result, controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (auto i = 0u; i < joint_command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        joint_name_, interface_names_[i], &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }
    // TODO(anyone): Add other command interfaces, if any

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));

    if (set_parameters)
    {
      controller_->get_node()->set_parameter({"joint", joint_name_});
      controller_->get_node()->set_parameter({"interface_names", interface_names_});
    }
  }

  void publish_commands()
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
    msg.header.frame_id = "twist_frame_id";
    msg.header.stamp = controller_->get_node()->get_clock()->now();
    msg.twist.linear.x = msg.twist.linear.y = msg.twist.linear.z = 10.0;
    msg.twist.angular.x = msg.twist.angular.y = msg.twist.angular.z = 0.1;

    command_publisher_->publish(msg);
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  std::string joint_name_ = "tcp";
  std::vector<std::string> interface_names_ = {"linear.x",  "linear.y",  "linear.z",
                                               "angular.x", "angular.y", "angular.z"};
  std::array<double, 6> joint_command_values_ = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};

  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableTwistController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerCommandMsg>::SharedPtr command_publisher_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class TwistControllerTestParameterizedParameters
: public TwistControllerTest,
  public ::testing::WithParamInterface<std::tuple<std::string, rclcpp::ParameterValue>>
{
public:
  virtual void SetUp() { TwistControllerTest::SetUp(); }

  static void TearDownTestCase() { TwistControllerTest::TearDownTestCase(); }

protected:
  void SetUpController(bool set_parameters = true)
  {
    TwistControllerTest::SetUpController(set_parameters);
    controller_->get_node()->set_parameter({std::get<0>(GetParam()), std::get<1>(GetParam())});
  }
};

#endif  // TEST_TWIST_CONTROLLER_HPP_
