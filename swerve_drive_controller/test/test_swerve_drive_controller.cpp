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

#include "test_swerve_drive_controller.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace swerve_drive_controller
{
void SwerveDriveControllerTest::SetUp()
{
  node_ = std::make_shared<rclcpp::Node>("test_node");
  RCLCPP_INFO(node_->get_logger(), "Test node initialized");

  // Declare parameters
  node_->declare_parameter("joint_steering_left_front", "front_left_axle_joint");
  node_->declare_parameter("joint_steering_right_front", "front_right_axle_joint");
  node_->declare_parameter("joint_steering_left_rear", "rear_left_axle_joint");
  node_->declare_parameter("joint_steering_right_rear", "rear_right_axle_joint");
  node_->declare_parameter("joint_wheel_left_front", "front_left_wheel_joint");
  node_->declare_parameter("joint_wheel_right_front", "front_right_wheel_joint");
  node_->declare_parameter("joint_wheel_left_rear", "rear_left_wheel_joint");
  node_->declare_parameter("joint_wheel_right_rear", "rear_right_wheel_joint");
  node_->declare_parameter("cmd_vel_topic", "~/cmd_vel");
  node_->declare_parameter("odom", "~/odom");
  node_->declare_parameter("base_footprint", "base_footprint");
  node_->declare_parameter("enable_odom_tf", true);
  node_->declare_parameter("open_loop", false);
  node_->declare_parameter("use_stamped_vel", true);
  node_->declare_parameter("publish_rate", 50.0);
  node_->declare_parameter("cmd_vel_timeout", 0.5);
  node_->declare_parameter("chassis_length", 0.5);
  node_->declare_parameter("chassis_width", 0.5);
  node_->declare_parameter("wheel_radius", 0.1);
  node_->declare_parameter("center_of_rotation", 0.0);
  node_->declare_parameter("front_left_velocity_threshold", 10.0);
  node_->declare_parameter("front_right_velocity_threshold", 10.0);
  node_->declare_parameter("rear_left_velocity_threshold", 10.0);
  node_->declare_parameter("rear_right_velocity_threshold", 10.0);

  // Define expected interfaces
  command_interfaces_ = {"front_left_wheel_joint/velocity", "front_right_wheel_joint/velocity",
                         "rear_left_wheel_joint/velocity",  "rear_right_wheel_joint/velocity",
                         "front_left_axle_joint/position",  "front_right_axle_joint/position",
                         "rear_left_axle_joint/position",   "rear_right_axle_joint/position"};
  state_interfaces_ = {"front_left_wheel_joint/velocity", "front_right_wheel_joint/velocity",
                       "rear_left_wheel_joint/velocity",  "rear_right_wheel_joint/velocity",
                       "front_left_axle_joint/position",  "front_right_axle_joint/position",
                       "rear_left_axle_joint/position",   "rear_right_axle_joint/position"};
}

void SwerveDriveControllerTest::SetUpController()
{
  RCLCPP_INFO(node_->get_logger(), "Creating SwerveController instance");
  controller_ = std::make_shared<SwerveController>();
  if (!controller_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create SwerveController instance");
    GTEST_FAIL() << "Controller creation failed";
  }
  RCLCPP_INFO(node_->get_logger(), "SwerveController instance created");

  // Initialize the controller's node
  RCLCPP_INFO(node_->get_logger(), "Initializing controller node");
  auto init_result = controller_->init(
    "swerve_controller_test",  // controller_name
    "",                        // urdf (empty for test)
    100,                       // cm_update_rate (100 Hz)
    "",                        // node_namespace (default)
    rclcpp::NodeOptions()      // node_options (default)
  );
  if (init_result != controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "Controller node initialization failed");
    GTEST_FAIL() << "Controller node initialization failed";
  }
  RCLCPP_INFO(node_->get_logger(), "Controller node initialized");
}

void SwerveDriveControllerTest::SetUpInterfaces()
{
  command_values_.resize(command_interfaces_.size(), 0.0);
  state_values_.resize(state_interfaces_.size(), 0.0);
  command_interfaces_base_.reserve(command_interfaces_.size());
  state_interfaces_base_.reserve(state_interfaces_.size());

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    const auto & name = command_interfaces_[i];
    auto interface_type = name.substr(name.find_last_of("/") + 1);
    command_interfaces_base_.emplace_back(
      hardware_interface::CommandInterface(
        name.substr(0, name.find_last_of("/")), interface_type, &command_values_[i]));
    command_interface_handles_.emplace_back(command_interfaces_base_.back());
  }
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    const auto & name = state_interfaces_[i];
    auto interface_type = name.substr(name.find_last_of("/") + 1);
    state_interfaces_base_.emplace_back(
      hardware_interface::StateInterface(
        name.substr(0, name.find_last_of("/")), interface_type, &state_values_[i]));
    state_interface_handles_.emplace_back(state_interfaces_base_.back());
  }
  controller_->assign_interfaces(
    std::move(command_interface_handles_), std::move(state_interface_handles_));
}

void SwerveDriveControllerTest::PublishTwistStamped(
  double linear_x, double linear_y, double angular_z)
{
  auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  msg->header.stamp = node_->now();
  msg->twist.linear.x = linear_x;
  msg->twist.linear.y = linear_y;
  msg->twist.angular.z = angular_z;
  twist_stamped_pub_->publish(*msg);
}

void SwerveDriveControllerTest::PublishTwist(double linear_x, double linear_y, double angular_z)
{
  auto msg = std::make_shared<geometry_msgs::msg::Twist>();
  msg->linear.x = linear_x;
  msg->linear.y = linear_y;
  msg->angular.z = angular_z;
  twist_pub_->publish(*msg);
}

TEST_F(SwerveDriveControllerTest, test_on_init_success)
{
  SetUpController();
  RCLCPP_INFO(node_->get_logger(), "Calling on_init");
  auto result = controller_->on_init();
  EXPECT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
  RCLCPP_INFO(node_->get_logger(), "on_init returned %d", static_cast<int>(result));
  RCLCPP_INFO(node_->get_logger(), "on_init completed with result: %d", static_cast<int>(result));
  init_successful_ = (result == controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_command_interface_configuration)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  auto config = controller_->command_interface_configuration();
  EXPECT_EQ(config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(config.names.size(), 8u);
  for (const auto & name : command_interfaces_)
  {
    EXPECT_NE(std::find(config.names.begin(), config.names.end(), name), config.names.end())
      << "Expected command interface " << name << " not found";
  }
}

TEST_F(SwerveDriveControllerTest, test_state_interface_configuration)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  auto config = controller_->state_interface_configuration();
  EXPECT_EQ(config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(config.names.size(), 8u);
  for (const auto & name : state_interfaces_)
  {
    EXPECT_NE(std::find(config.names.begin(), config.names.end(), name), config.names.end())
      << "Expected state interface " << name << " not found";
  }
}

TEST_F(SwerveDriveControllerTest, test_on_configure_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  EXPECT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_on_configure_missing_parameters)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  node_->undeclare_parameter("joint_wheel_left_front");
  EXPECT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveDriveControllerTest, test_on_activate_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_on_activate_missing_interfaces)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(SwerveDriveControllerTest, test_on_deactivate_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_on_cleanup_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->on_cleanup(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_on_error_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->on_error(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_on_shutdown_success)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  EXPECT_EQ(
    controller_->on_shutdown(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, test_update_inactive_state)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.02)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(command_values_[i], 0.0) << "Wheel velocity " << i << " not zeroed";
  }
  for (size_t i = 4; i < 8; ++i)
  {
    EXPECT_EQ(command_values_[i], 0.0) << "Axle position " << i << " not zeroed";
  }
}

TEST_F(SwerveDriveControllerTest, test_update_with_velocity_command)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Set up odometry subscriber
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "~/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_msg_ = msg; });

  // Set up velocity publisher
  twist_stamped_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS());

  // Publish a velocity command (1 m/s forward)
  PublishTwistStamped(1.0, 0.0, 0.0);
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Update controller
  auto time = node_->now();
  EXPECT_EQ(
    controller_->update(time, rclcpp::Duration::from_seconds(0.02)),
    controller_interface::return_type::OK);

  // Spin to receive odometry
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  ASSERT_NE(last_odom_msg_, nullptr);
  EXPECT_NEAR(last_odom_msg_->pose.pose.position.x, 0.02, 0.01);  // 1 m/s * 0.02 s
  EXPECT_NEAR(last_odom_msg_->pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(last_odom_msg_->pose.pose.orientation.z, 0.0, 0.01);
}

TEST_F(SwerveDriveControllerTest, test_update_with_timeout)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Set up velocity publisher
  twist_stamped_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS());
  PublishTwistStamped(1.0, 0.0, 0.0);
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Wait beyond timeout (0.5 s)
  rclcpp::sleep_for(std::chrono::milliseconds(600));

  // Update controller
  EXPECT_EQ(
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.02)),
    controller_interface::return_type::OK);

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_EQ(command_values_[i], 0.0) << "Wheel velocity " << i << " not zeroed";
  }
  for (size_t i = 4; i < 8; ++i)
  {
    EXPECT_EQ(command_values_[i], 0.0) << "Axle position " << i << " not zeroed";
  }
}

TEST_F(SwerveDriveControllerTest, test_unstamped_velocity_command)
{
  SetUpController();
  if (!init_successful_)
  {
    GTEST_SKIP() << "Skipping due to on_init failure";
  }
  node_->set_parameter(rclcpp::Parameter("use_stamped_vel", false));
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetUpInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Set up publishers and subscribers
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "~/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_msg_ = msg; });

  // Publish an unstamped velocity command (1 m/s lateral)
  PublishTwist(0.0, 1.0, 0.0);
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Update controller
  auto time = node_->now();
  EXPECT_EQ(
    controller_->update(time, rclcpp::Duration::from_seconds(0.02)),
    controller_interface::return_type::OK);

  // Spin to receive odometry
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Check odometry
  ASSERT_NE(last_odom_msg_, nullptr);
  EXPECT_NEAR(last_odom_msg_->pose.pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(last_odom_msg_->pose.pose.position.y, 0.02, 0.01);  // 1 m/s * 0.02 s
  EXPECT_NEAR(last_odom_msg_->pose.pose.orientation.z, 0.0, 0.01);
}

}  // namespace swerve_drive_controller

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
