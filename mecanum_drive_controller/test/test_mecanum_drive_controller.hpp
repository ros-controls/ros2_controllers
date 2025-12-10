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

#ifndef TEST_MECANUM_DRIVE_CONTROLLER_HPP_
#define TEST_MECANUM_DRIVE_CONTROLLER_HPP_

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
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using ControllerStateMsg = mecanum_drive_controller::MecanumDriveController::ControllerStateMsg;
using ControllerReferenceMsg =
  mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg;
using TfStateMsg = mecanum_drive_controller::MecanumDriveController::TfStateMsg;
using OdomStateMsg = mecanum_drive_controller::MecanumDriveController::OdomStateMsg;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableMecanumDriveController : public mecanum_drive_controller::MecanumDriveController
{
  FRIEND_TEST(MecanumDriveControllerTest, when_controller_is_configured_expect_all_parameters_set);
  FRIEND_TEST(
    MecanumDriveControllerTest, when_controller_configured_expect_properly_exported_interfaces);
  FRIEND_TEST(MecanumDriveControllerTest, when_controller_is_activated_expect_reference_reset);
  FRIEND_TEST(MecanumDriveControllerTest, when_controller_active_and_update_called_expect_success);
  FRIEND_TEST(MecanumDriveControllerTest, when_active_controller_is_deactivated_expect_success);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_controller_is_reactivated_expect_cmd_itfs_not_set_and_update_success);
  FRIEND_TEST(MecanumDriveControllerTest, when_update_is_called_expect_status_message);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_reference_msg_received_expect_updated_commands_and_status_message);
  FRIEND_TEST(MecanumDriveControllerTest, when_reference_msg_is_too_old_expect_unset_reference);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_reference_msg_has_timestamp_zero_expect_reference_set_and_timestamp_set_to_current_time);
  FRIEND_TEST(MecanumDriveControllerTest, when_message_has_valid_timestamp_expect_reference_set);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_ref_msg_old_expect_cmnd_itfs_set_to_zero_otherwise_to_valid_cmnds);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_controller_in_chainable_mode_expect_receiving_commands_from_reference_interfaces_directly);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_reference_timeout_is_zero_expect_reference_msg_being_used_only_once);
  FRIEND_TEST(
    MecanumDriveControllerTest,
    when_ref_timeout_zero_for_reference_callback_expect_reference_msg_being_used_only_once);
  FRIEND_TEST(MecanumDriveControllerTest, SideToSideAndRotationOdometryTest);

  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_false_no_namespace);
  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_true_no_namespace);
  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_blank_prefix_true_no_namespace);
  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_false_set_namespace);
  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_test_prefix_true_set_namespace);
  FRIEND_TEST(MecanumDriveControllerTest, configure_succeeds_tf_blank_prefix_true_set_namespace);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return mecanum_drive_controller::MecanumDriveController::on_configure(previous_state);
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return mecanum_drive_controller::MecanumDriveController::on_activate(previous_state);
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

  size_t get_front_left_wheel_index() { return WheelIndex::FRONT_LEFT; }

  size_t get_front_right_wheel_index() { return WheelIndex::FRONT_RIGHT; }

  size_t get_rear_right_wheel_index() { return WheelIndex::REAR_RIGHT; }

  size_t get_rear_left_wheel_index() { return WheelIndex::REAR_LEFT; }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class MecanumDriveControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_mecanum_drive_controller/reference", rclcpp::SystemDefaultsQoS());

    odom_s_publisher_node_ = std::make_shared<rclcpp::Node>("odom_s_publisher");
    odom_s_publisher_ = odom_s_publisher_node_->create_publisher<OdomStateMsg>(
      "/test_mecanum_drive_controller/odometry", rclcpp::SystemDefaultsQoS());

    tf_odom_s_publisher_node_ = std::make_shared<rclcpp::Node>("tf_odom_s_publisher");
    tf_odom_s_publisher_ = tf_odom_s_publisher_node_->create_publisher<TfStateMsg>(
      "/test_mecanum_drive_controller/tf_odometry", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(
    const std::string controller_name = "test_mecanum_drive_controller",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(), const std::string ns = "")
  {
    const auto urdf = "";
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = controller_name;
    params.robot_description = urdf;
    params.update_rate = 0;
    params.node_namespace = ns;
    params.node_options = node_options;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (size_t i = 0; i < joint_command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          command_joint_names_[i], interface_name_, &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    for (size_t i = 0; i < joint_state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          command_joint_names_[i], interface_name_, &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_to_controller_status_execute_update_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    ControllerStateMsg::SharedPtr received_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr cb_msg) { received_msg = cb_msg; };
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_mecanum_drive_controller/controller_state", 10, subs_callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());
    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
      const auto timeout = std::chrono::milliseconds{5};
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
    const rclcpp::Time & stamp, const double & twist_linear_x = 1.5,
    const double & twist_linear_y = 0.0, const double & twist_angular_z = 0.0)
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.header.stamp = stamp;
    msg.twist.linear.x = twist_linear_x;
    msg.twist.linear.y = twist_linear_y;
    msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.z = twist_angular_z;

    command_publisher_->publish(msg);
  }

protected:
  std::vector<std::string> reference_interface_names = {"linear/x", "linear/y", "angular/z"};

  static constexpr char TEST_FRONT_LEFT_CMD_JOINT_NAME[] = "front_left_wheel_joint";
  static constexpr char TEST_FRONT_RIGHT_CMD_JOINT_NAME[] = "front_right_wheel_joint";
  static constexpr char TEST_REAR_RIGHT_CMD_JOINT_NAME[] = "back_right_wheel_joint";
  static constexpr char TEST_REAR_LEFT_CMD_JOINT_NAME[] = "back_left_wheel_joint";
  std::vector<std::string> command_joint_names_ = {
    TEST_FRONT_LEFT_CMD_JOINT_NAME, TEST_FRONT_RIGHT_CMD_JOINT_NAME, TEST_REAR_RIGHT_CMD_JOINT_NAME,
    TEST_REAR_LEFT_CMD_JOINT_NAME};

  static constexpr char TEST_FRONT_LEFT_STATE_JOINT_NAME[] = "state_front_left_wheel_joint";
  static constexpr char TEST_FRONT_RIGHT_STATE_JOINT_NAME[] = "state_front_right_wheel_joint";
  static constexpr char TEST_REAR_RIGHT_STATE_JOINT_NAME[] = "state_back_right_wheel_joint";
  static constexpr char TEST_REAR_LEFT_STATE_JOINT_NAME[] = "state_back_left_wheel_joint";
  std::vector<std::string> state_joint_names_ = {
    TEST_FRONT_LEFT_STATE_JOINT_NAME, TEST_FRONT_RIGHT_STATE_JOINT_NAME,
    TEST_REAR_RIGHT_STATE_JOINT_NAME, TEST_REAR_LEFT_STATE_JOINT_NAME};
  std::string interface_name_ = hardware_interface::HW_IF_VELOCITY;

  // Controller-related parameters

  std::array<double, 4> joint_state_values_ = {{0.1, 0.1, 0.1, 0.1}};
  std::array<double, 4> joint_command_values_ = {{101.101, 101.101, 101.101, 101.101}};

  static constexpr double TEST_LINEAR_VELOCITY_X = 1.5;
  static constexpr double TEST_LINEAR_VELOCITY_y = 0.0;
  static constexpr double TEST_ANGULAR_VELOCITY_Z = 0.0;
  double command_lin_x = 111;

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  double ref_timeout_ = 0.1;

  // Test related parameters
  std::unique_ptr<CtrlType> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr odom_s_publisher_node_;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_;
  rclcpp::Node::SharedPtr tf_odom_s_publisher_node_;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
};

#endif  // TEST_MECANUM_DRIVE_CONTROLLER_HPP_
