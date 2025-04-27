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

#ifndef TEST_BICYCLE_STEERING_CONTROLLER_HPP_
#define TEST_BICYCLE_STEERING_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "bicycle_steering_controller/bicycle_steering_controller.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using ControllerStateMsg =
  steering_controllers_library::SteeringControllersLibrary::SteeringControllerStateMsg;
using ControllerReferenceMsg =
  steering_controllers_library::SteeringControllersLibrary::ControllerTwistReferenceMsg;

// name constants for state interfaces
using bicycle_steering_controller::STATE_STEER_AXIS;
using bicycle_steering_controller::STATE_TRACTION_WHEEL;

// name constants for command interfaces
using bicycle_steering_controller::CMD_STEER_WHEEL;
using bicycle_steering_controller::CMD_TRACTION_WHEEL;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
const double COMMON_THRESHOLD = 1e-6;

}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableBicycleSteeringController
: public bicycle_steering_controller::BicycleSteeringController
{
  FRIEND_TEST(BicycleSteeringControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(BicycleSteeringControllerTest, check_exported_interfaces);
  FRIEND_TEST(BicycleSteeringControllerTest, activate_success);
  FRIEND_TEST(BicycleSteeringControllerTest, update_success);
  FRIEND_TEST(BicycleSteeringControllerTest, deactivate_success);
  FRIEND_TEST(BicycleSteeringControllerTest, reactivate_success);
  FRIEND_TEST(BicycleSteeringControllerTest, test_update_logic);
  FRIEND_TEST(BicycleSteeringControllerTest, test_update_logic_chained);
  FRIEND_TEST(BicycleSteeringControllerTest, publish_status_success);
  FRIEND_TEST(BicycleSteeringControllerTest, receive_message_and_publish_updated_status);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return bicycle_steering_controller::BicycleSteeringController::on_configure(previous_state);
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return bicycle_steering_controller::BicycleSteeringController::on_activate(previous_state);
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
    wait_for_command(executor, timeout);
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class BicycleSteeringControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_bicycle_steering_controller/reference", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_bicycle_steering_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    if (position_feedback_ == true)
    {
      traction_interface_name_ = "position";
    }
    else
    {
      traction_interface_name_ = "velocity";
    }

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    command_itfs_.emplace_back(
      hardware_interface::CommandInterface(
        rear_wheels_names_[0], traction_interface_name_,
        &joint_command_values_[CMD_TRACTION_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    command_itfs_.emplace_back(
      hardware_interface::CommandInterface(
        front_wheels_names_[0], steering_interface_name_, &joint_command_values_[CMD_STEER_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    state_itfs_.emplace_back(
      hardware_interface::StateInterface(
        rear_wheels_names_[0], traction_interface_name_,
        &joint_state_values_[STATE_TRACTION_WHEEL]));
    state_ifs.emplace_back(state_itfs_.back());

    state_itfs_.emplace_back(
      hardware_interface::StateInterface(
        front_wheels_names_[0], steering_interface_name_, &joint_state_values_[STATE_STEER_AXIS]));
    state_ifs.emplace_back(state_itfs_.back());

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    ControllerStateMsg::SharedPtr received_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr cb_msg) { received_msg = cb_msg; };
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_bicycle_steering_controller/controller_state", 10, subs_callback);
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

  void publish_commands(const double linear = 0.1, const double angular = 0.2)
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

    ControllerReferenceMsg msg;
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;

    command_publisher_->publish(msg);
  }

protected:
  // Controller-related parameters
  double reference_timeout_ = 2.0;
  bool front_steering_ = true;
  bool open_loop_ = false;
  unsigned int velocity_rolling_window_size_ = 10;
  bool position_feedback_ = false;
  bool use_stamped_vel_ = true;
  std::vector<std::string> rear_wheels_names_ = {"rear_wheel_joint"};
  std::vector<std::string> front_wheels_names_ = {"steering_axis_joint"};
  std::vector<std::string> joint_names_ = {rear_wheels_names_[0], front_wheels_names_[0]};

  std::vector<std::string> rear_wheels_preceding_names_ = {"pid_controller/rear_wheel_joint"};
  std::vector<std::string> front_wheels_preceding_names_ = {"pid_controller/steering_axis_joint"};
  std::vector<std::string> preceding_joint_names_ = {
    rear_wheels_preceding_names_[0], front_wheels_preceding_names_[0]};

  double wheelbase_ = 3.24644;
  double front_wheels_radius_ = 0.45;
  double rear_wheels_radius_ = 0.45;

  std::array<double, 2> joint_state_values_ = {3.3, 0.5};
  std::array<double, 2> joint_command_values_ = {1.1, 2.2};
  std::array<std::string, 2> joint_reference_interfaces_ = {"linear/velocity", "angular/velocity"};
  std::string steering_interface_name_ = "position";

  // defined in setup
  std::string traction_interface_name_ = "";
  std::string preceding_prefix_ = "pid_controller";

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableBicycleSteeringController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
};

#endif  // TEST_BICYCLE_STEERING_CONTROLLER_HPP_
