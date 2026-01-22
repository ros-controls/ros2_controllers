// Copyright 2025 Aarav Gupta
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

#ifndef TEST_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_
#define TEST_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest_prod.h>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "omni_wheel_drive_controller/omni_wheel_drive_controller.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
std::vector<std::string> wheel_names_ = {
  "front_wheel_joint", "left_wheel_joint", "back_wheel_joint", "right_wheel_joint"};
}  // namespace

// subclassing and friending so we can access member variables
class TestableOmniWheelDriveController
: public omni_wheel_drive_controller::OmniWheelDriveController
{
  FRIEND_TEST(OmniWheelDriveControllerTest, configure_succeeds_tf_prefix_no_namespace);
  FRIEND_TEST(OmniWheelDriveControllerTest, configure_succeeds_tf_blank_prefix_no_namespace);
  FRIEND_TEST(OmniWheelDriveControllerTest, configure_succeeds_tf_prefix_set_namespace);
  FRIEND_TEST(OmniWheelDriveControllerTest, configure_succeeds_tf_tilde_prefix_set_namespace);
  FRIEND_TEST(OmniWheelDriveControllerTest, cleanup);
  FRIEND_TEST(OmniWheelDriveControllerTest, chainable_controller_unchained_mode);
  FRIEND_TEST(OmniWheelDriveControllerTest, chainable_controller_chained_mode);
  FRIEND_TEST(OmniWheelDriveControllerTest, deactivate_then_activate);
  FRIEND_TEST(OmniWheelDriveControllerTest, command_with_zero_timestamp_is_accepted_with_warning);
  FRIEND_TEST(OmniWheelDriveControllerTest, 3_wheel_test);
  FRIEND_TEST(OmniWheelDriveControllerTest, 3_wheel_rot_test);
  FRIEND_TEST(OmniWheelDriveControllerTest, 4_wheel_rot_test);
  FRIEND_TEST(OmniWheelDriveControllerTest, 5_wheel_test);

  /**
   * @brief wait_for_twist block until a new twist is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_twist(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class OmniWheelDriveControllerFixture : public ::testing::Test
{
public:
  void SetUp()
  {
    // Initialize controller
    controller_ = std::make_unique<CtrlType>();

    cmd_vel_publisher_node_ = std::make_shared<rclcpp::Node>("cmd_vel_publisher");
    cmd_vel_publisher_ =
      cmd_vel_publisher_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/test_omni_wheel_drive_controller/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

protected:
  void publish_twist(double linear_x = 1.0, double linear_y = 1.0, double angular = 1.0)
  {
    publish_twist_timestamped(
      cmd_vel_publisher_node_->get_clock()->now(), linear_x, linear_y, angular);
  }

  void publish_twist_timestamped(
    const rclcpp::Time & stamp, const double & twist_linear_x = 1.0,
    const double & twist_linear_y = 1.0, const double & twist_angular_z = 1.0)
  {
    const char * topic_name = cmd_vel_publisher_->get_topic_name();
    size_t wait_count = 0;
    while (cmd_vel_publisher_node_->count_subscribers(topic_name) == 0)
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

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = stamp;
    msg.twist.linear.x = twist_linear_x;
    msg.twist.linear.y = twist_linear_y;
    msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.z = twist_angular_z;

    cmd_vel_publisher_->publish(msg);
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void waitForSetup(rclcpp::Executor & executor)
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = cmd_vel_publisher_node_->get_clock();
    auto start = clock->now();
    while (cmd_vel_publisher_->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL();
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void assignResourcesPosFeedback(const std::vector<std::string> wheel_names = wheel_names_)
  {
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_ifs;
    loaned_state_ifs.reserve(wheels_pos_states_.size());
    state_itfs_.reserve(wheels_pos_states_.size());
    for (size_t i = 0; i < wheels_pos_states_.size(); ++i)
    {
      state_itfs_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(
          wheel_names[i], HW_IF_POSITION, &wheels_pos_states_[i]));
      loaned_state_ifs.emplace_back(state_itfs_.back(), nullptr);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_ifs;
    loaned_command_ifs.reserve(wheels_vel_cmds_.size());
    command_itfs_.reserve(wheels_vel_cmds_.size());
    for (size_t i = 0; i < wheels_vel_cmds_.size(); ++i)
    {
      command_itfs_.emplace_back(
        std::make_shared<hardware_interface::CommandInterface>(
          wheel_names[i], HW_IF_VELOCITY, &wheels_vel_cmds_[i]));
      loaned_command_ifs.emplace_back(command_itfs_.back(), nullptr);
    }

    controller_->assign_interfaces(std::move(loaned_command_ifs), std::move(loaned_state_ifs));
  }

  void assignResourcesVelFeedback()
  {
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_ifs;
    loaned_state_ifs.reserve(wheels_vel_states_.size());
    state_itfs_.reserve(wheels_vel_states_.size());
    for (size_t i = 0; i < wheels_vel_states_.size(); ++i)
    {
      state_itfs_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(
          wheel_names_[i], HW_IF_VELOCITY, &wheels_vel_states_[i]));
      loaned_state_ifs.emplace_back(state_itfs_.back(), nullptr);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_ifs;
    loaned_command_ifs.reserve(wheels_vel_cmds_.size());
    command_itfs_.reserve(wheels_vel_cmds_.size());
    for (size_t i = 0; i < wheels_vel_cmds_.size(); ++i)
    {
      command_itfs_.emplace_back(
        std::make_shared<hardware_interface::CommandInterface>(
          wheel_names_[i], HW_IF_VELOCITY, &wheels_vel_cmds_[i]));
      loaned_command_ifs.emplace_back(command_itfs_.back(), nullptr);
    }

    controller_->assign_interfaces(std::move(loaned_command_ifs), std::move(loaned_state_ifs));
  }

  controller_interface::return_type InitController(
    const std::vector<std::string> wheel_joints_init = wheel_names_,
    const double wheel_offset = 0.0, const std::vector<rclcpp::Parameter> & parameters = {},
    const std::string ns = "")
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    parameter_overrides.push_back(
      rclcpp::Parameter("wheel_names", rclcpp::ParameterValue(wheel_joints_init)));
    parameter_overrides.push_back(
      rclcpp::Parameter("wheel_offset", rclcpp::ParameterValue(wheel_offset)));
    parameter_overrides.push_back(rclcpp::Parameter("robot_radius", rclcpp::ParameterValue(0.5)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.1)));

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = controller_name_;
    params.robot_description = urdf_;
    params.update_rate = 0;
    params.node_namespace = ns;
    params.node_options = node_options;
    return controller_->init(params);
  }

  std::vector<double> wheels_pos_states_ = {1, 1, 1, 1};
  std::vector<double> wheels_vel_states_ = {1, 1, 1, 1};
  std::vector<double> wheels_vel_cmds_ = {0.1, 0.2, 0.3, 0.4};

  std::vector<hardware_interface::StateInterface::SharedPtr> state_itfs_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_itfs_;

  std::vector<std::string> reference_interface_names = {"linear/x", "linear/y", "angular/z"};

  std::unique_ptr<CtrlType> controller_;
  rclcpp::Node::SharedPtr cmd_vel_publisher_node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

  const std::string urdf_ = "";
  std::string controller_name_ = "test_omni_wheel_drive_controller";
};

#endif  // TEST_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_
