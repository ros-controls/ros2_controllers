// Copyright 2025 ros2_control development team
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

#ifndef TEST_SWERVE_DRIVE_CONTROLLER_HPP_
#define TEST_SWERVE_DRIVE_CONTROLLER_HPP_

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
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "swerve_drive_controller/swerve_drive_controller.hpp"

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace swerve_drive_controller
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;

std::vector<std::string> wheel_joint_names_ = {
  "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
  "rear_right_wheel_joint"};
std::vector<std::string> steering_joint_names_ = {
  "front_left_axle_joint", "front_right_axle_joint", "rear_left_axle_joint",
  "rear_right_axle_joint"};

class TestableSwerveDriveController : public SwerveController
{
  FRIEND_TEST(SwerveDriveControllerTest, init_fails_without_parameters);
  FRIEND_TEST(SwerveDriveControllerTest, configure_fails_with_missing_wheels);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_no_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_with_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_prefix_false_no_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_prefix_true_no_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_blank_prefix_true_no_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_prefix_false_set_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_prefix_true_set_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, configure_succeeds_tf_blank_prefix_true_set_namespace);
  FRIEND_TEST(SwerveDriveControllerTest, activate_fails_without_resources_assigned);
  FRIEND_TEST(SwerveDriveControllerTest, activate_succeeds_with_resources_assigned);
  FRIEND_TEST(SwerveDriveControllerTest, deactivate_then_activate);
  FRIEND_TEST(SwerveDriveControllerTest, command_with_zero_timestamp_is_accepted_with_warning);

public:
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  get_realtime_odometry_publisher() const
  {
    return realtime_odometry_publisher_;
  }

  /**
   * @brief wait_for_twist block until a new twist is received.
   * Requires that the executor is not spinned elsewhere between the
   * message publication and the call to this function
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

template <typename CtrlType>
class SwerveDriveControllerFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    controller_ = std::make_unique<CtrlType>();

    cmd_vel_publisher_node_ = std::make_shared<rclcpp::Node>("cmd_vel_publisher");
    cmd_vel_publisher_ =
      cmd_vel_publisher_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/test_swerve_drive_controller/cmd_vel", rclcpp::SystemDefaultsQoS());

    odom_subscriber_node_ = std::make_shared<rclcpp::Node>("odom_subscriber");
    odom_sub_ = odom_subscriber_node_->create_subscription<nav_msgs::msg::Odometry>(
      "/test_swerve_drive_controller/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_msg_ = msg; });
  }

  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

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
  void waitForSetup()
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
      rclcpp::spin_some(cmd_vel_publisher_node_);
    }
  }

  void assignResources()
  {
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_ifs.reserve(wheel_vel_states_.size() + steering_pos_states_.size());
    state_itfs_.reserve(wheel_vel_states_.size() + steering_pos_states_.size());
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_ifs.reserve(wheel_vel_cmds_.size() + steering_pos_cmds_.size());
    command_itfs_.reserve(wheel_vel_cmds_.size() + steering_pos_cmds_.size());

    // Wheel velocity interfaces
    for (size_t i = 0; i < wheel_vel_states_.size(); ++i)
    {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          wheel_joint_names_[i], HW_IF_VELOCITY, &wheel_vel_states_[i]));
      state_ifs.emplace_back(state_itfs_.back());
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          wheel_joint_names_[i], HW_IF_VELOCITY, &wheel_vel_cmds_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    // Steering position interfaces
    for (size_t i = 0; i < steering_pos_states_.size(); ++i)
    {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          steering_joint_names_[i], HW_IF_POSITION, &steering_pos_states_[i]));
      state_ifs.emplace_back(state_itfs_.back());
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          steering_joint_names_[i], HW_IF_POSITION, &steering_pos_cmds_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(
    const std::vector<std::string> & wheel_joints = wheel_joint_names_,
    const std::vector<std::string> & steering_joints = steering_joint_names_,
    const std::vector<rclcpp::Parameter> & parameters = {}, const std::string & ns = "")
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    if (wheel_joints.size() > 0)
    {
      parameter_overrides.push_back(rclcpp::Parameter("front_left_wheel_joint", wheel_joints[0]));
    }
    if (wheel_joints.size() > 1)
    {
      parameter_overrides.push_back(rclcpp::Parameter("front_right_wheel_joint", wheel_joints[1]));
    }
    if (wheel_joints.size() > 2)
    {
      parameter_overrides.push_back(rclcpp::Parameter("rear_left_wheel_joint", wheel_joints[2]));
    }
    if (wheel_joints.size() > 3)
    {
      parameter_overrides.push_back(rclcpp::Parameter("rear_right_wheel_joint", wheel_joints[3]));
    }
    if (steering_joints.size() > 0)
    {
      parameter_overrides.push_back(rclcpp::Parameter("front_left_axle_joint", steering_joints[0]));
    }
    if (steering_joints.size() > 1)
    {
      parameter_overrides.push_back(
        rclcpp::Parameter("front_right_axle_joint", steering_joints[1]));
    }
    if (steering_joints.size() > 2)
    {
      parameter_overrides.push_back(rclcpp::Parameter("rear_left_axle_joint", steering_joints[2]));
    }
    if (steering_joints.size() > 3)
    {
      parameter_overrides.push_back(rclcpp::Parameter("rear_right_axle_joint", steering_joints[3]));
    }

    if (wheel_joints.size() >= 4 && steering_joints.size() >= 4)
    {
      parameter_overrides.push_back(
        rclcpp::Parameter("chassis_length", rclcpp::ParameterValue(0.2)));
      parameter_overrides.push_back(
        rclcpp::Parameter("chassis_width", rclcpp::ParameterValue(0.35)));
      parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.1)));
      parameter_overrides.push_back(
        rclcpp::Parameter("cmd_vel_timeout", rclcpp::ParameterValue(0.5)));
      parameter_overrides.push_back(rclcpp::Parameter("odom", rclcpp::ParameterValue("odom")));
      parameter_overrides.push_back(
        rclcpp::Parameter("base_footprint", rclcpp::ParameterValue("base_footprint")));
      parameter_overrides.push_back(rclcpp::Parameter("open_loop", rclcpp::ParameterValue(false)));
      parameter_overrides.push_back(
        rclcpp::Parameter("center_of_rotation", rclcpp::ParameterValue(0.1)));
    }

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    return controller_->init("test_swerve_drive_controller", "", 0, ns, node_options);
  }

  std::vector<double> wheel_vel_states_ = {1.0, 1.0, 1.0, 1.0};
  std::vector<double> wheel_vel_cmds_ = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> steering_pos_states_ = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> steering_pos_cmds_ = {0.0, 0.0, 0.0, 0.0};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  std::unique_ptr<CtrlType> controller_;
  rclcpp::Node::SharedPtr cmd_vel_publisher_node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::Node::SharedPtr odom_subscriber_node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  const std::string urdf_ = "";
  std::string controller_name_ = "test_swerve_drive_controller";
};
}  // namespace swerve_drive_controller
#endif  // TEST_SWERVE_DRIVE_CONTROLLER_HPP_
