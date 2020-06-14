// Copyright 2020 PAL Robotics SL.
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

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

#include "gtest/gtest.h"
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using lifecycle_msgs::msg::State;

void spin(rclcpp::executors::MultiThreadedExecutor * exe)
{
  exe->spin();
}

class TestDiffDriveController : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    test_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
    test_robot->init();
    left_wheel_names =
    {{test_robot->joint_name1, test_robot->joint_name2, test_robot->joint_name3}};
    right_wheel_names =
    {{test_robot->joint_name1, test_robot->joint_name2, test_robot->joint_name3}};
    op_mode = {{test_robot->write_op_handle_name1}};

    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::Twist>(
      controller_name + "/cmd_vel",
      rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  /// Publish velocity msgs
  /**
   *  linear - magnitude of the linear command in the geometry_msgs::twist message
   *  angular - the magnitude of the angular command in geometry_msgs::twist message
   */
  void publish(double linear, double angular)
  {
    int wait_count = 0;
    auto topic = velocity_publisher->get_topic_name();
    while (pub_node->count_subscribers(topic) == 0) {
      if (wait_count >= 5) {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    geometry_msgs::msg::Twist velocity_message;
    velocity_message.linear.x = linear;
    velocity_message.angular.z = angular;
    velocity_publisher->publish(velocity_message);
  }

  std::string controller_name = "test_diff_drive_controller";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot;
  std::vector<std::string> left_wheel_names;
  std::vector<std::string> right_wheel_names;
  std::vector<std::string> op_mode;

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
};

TEST_F(TestDiffDriveController, wrong_initialization)
{
  auto uninitialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  auto diff_drive_controller =
    std::make_shared<diff_drive_controller::DiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(uninitialized_robot, controller_name);
  ASSERT_EQ(ret, controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS);

  auto unconfigured_state = diff_drive_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, unconfigured_state.id());
}

TEST_F(TestDiffDriveController, correct_initialization)
{
  auto initialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  initialized_robot->init();
  auto diff_drive_controller =
    std::make_shared<diff_drive_controller::DiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(initialized_robot, controller_name);
  ASSERT_EQ(ret, controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS);

  auto inactive_state = diff_drive_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, inactive_state.id());
  EXPECT_EQ(1.1, initialized_robot->pos1);
  EXPECT_EQ(2.2, initialized_robot->pos2);
  EXPECT_EQ(3.3, initialized_robot->pos3);
}

TEST_F(TestDiffDriveController, configuration)
{
  auto diff_drive_controller =
    std::make_shared<diff_drive_controller::DiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(test_robot, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    FAIL();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(diff_drive_controller->get_lifecycle_node()->get_node_base_interface());
  auto future_handle_ = std::async(std::launch::async, spin, &executor);

  auto state = diff_drive_controller->get_lifecycle_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // add in check for linear vel command values

  executor.cancel();
}

TEST_F(TestDiffDriveController, cleanup)
{
  auto diff_drive_controller =
    std::make_shared<diff_drive_controller::DiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(test_robot, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    FAIL();
  }

  auto diff_drive_lifecycle_node = diff_drive_controller->get_lifecycle_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(diff_drive_lifecycle_node->get_node_base_interface());

  auto state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  state = diff_drive_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // wait for the subscriber and publisher to completely setup
  constexpr std::chrono::seconds TIMEOUT{2};
  auto clock = pub_node->get_clock();
  auto start = clock->now();
  auto timedout = true;
  while (velocity_publisher->get_subscription_count() <= 0) {
    if ((clock->now() - start) > TIMEOUT) {
      timedout = false;
    }
    rclcpp::spin_some(pub_node);
  }
  ASSERT_TRUE(timedout);

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);

  // wait for msg is be published to the system
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  executor.spin_once();

  diff_drive_controller->update();
  test_robot->write();

  state = diff_drive_lifecycle_node->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  diff_drive_controller->update();
  test_robot->write();

  state = diff_drive_lifecycle_node->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  diff_drive_controller->update();
  test_robot->write();

  // shouild be stopped
  EXPECT_EQ(0, test_robot->cmd1);
  EXPECT_EQ(0, test_robot->cmd2);

  executor.cancel();
}

TEST_F(TestDiffDriveController, correct_initialization_using_parameters)
{
  auto diff_drive_controller =
    std::make_shared<diff_drive_controller::DiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(test_robot, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
    FAIL();
  }

  // This block is replacing the way parameters are set via launch
  auto diff_drive_lifecycle_node = diff_drive_controller->get_lifecycle_node();
  rclcpp::Parameter joint_parameters("left_wheel_names", left_wheel_names);
  diff_drive_lifecycle_node->set_parameter(joint_parameters);

  std::vector<std::string> operation_mode_names = {"motor_controls"};
  rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  diff_drive_lifecycle_node->set_parameter(operation_mode_parameters);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(diff_drive_lifecycle_node->get_node_base_interface());

  auto future_handle = std::async(std::launch::async, [&executor]() -> void {executor.spin();});

  auto state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(1.1, test_robot->cmd1);
  EXPECT_EQ(2.2, test_robot->cmd2);

  state = diff_drive_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // wait for the subscriber and publisher to completely setup
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);
  // wait for msg is be published to the system
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  diff_drive_controller->update();
  test_robot->write();

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = diff_drive_lifecycle_node->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  diff_drive_controller->update();
  test_robot->write();

  // no change in hw position
  EXPECT_EQ(1.1, test_robot->cmd1);
  EXPECT_EQ(2.2, test_robot->cmd2);

  // cleanup
  state = diff_drive_lifecycle_node->cleanup();
  diff_drive_controller->update();
  test_robot->write();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0, test_robot->cmd1);
  EXPECT_EQ(0, test_robot->cmd2);

  state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
