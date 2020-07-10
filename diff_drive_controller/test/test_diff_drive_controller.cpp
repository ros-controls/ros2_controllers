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

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"


using lifecycle_msgs::msg::State;

class TestableDiffDriveController : public diff_drive_controller::DiffDriveController
{
public:
  using DiffDriveController::DiffDriveController;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> getLastReceivedTwist() const
  {
    return received_velocity_msg_ptr_;
  }

  /**
  * @brief wait_for_twist block until a new twist is received.
  * Requires that the executor is not spinned elsewhere between the
  *  message publication and the call to this function
  *
  * @return true if new twist msg was received, false if timeout
  */
  bool wait_for_twist(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(velocity_command_subscriber_);

    if (wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready) {
      executor.spin_some();
      return true;
    }
    return false;
  }
};

class TestDiffDriveController : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    test_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
    test_robot->init();
    left_wheel_names =
    {{test_robot->joint_name1, test_robot->joint_name2, test_robot->joint_name3}};
    right_wheel_names =
    {{test_robot->joint_name1, test_robot->joint_name2, test_robot->joint_name3}};
    op_mode = {{test_robot->write_op_handle_name1}};

    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>(
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

    geometry_msgs::msg::TwistStamped velocity_message;
    velocity_message.header.stamp = pub_node->get_clock()->now();
    velocity_message.twist.linear.x = linear;
    velocity_message.twist.angular.z = angular;
    velocity_publisher->publish(velocity_message);
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void waitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = pub_node->get_clock();
    auto start = clock->now();
    while (velocity_publisher->get_subscription_count() <= 0) {
      if ((clock->now() - start) > TIMEOUT) {
        FAIL();
      }
      rclcpp::spin_some(pub_node);
    }
  }

  void setup_controller(
    TestableDiffDriveController & controller,
    rclcpp::Executor & executor)
  {
    auto diff_drive_lifecycle_node = controller.get_lifecycle_node();
    executor.add_node(diff_drive_lifecycle_node->get_node_base_interface());
    auto state = diff_drive_lifecycle_node->configure();
    ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

    state = diff_drive_lifecycle_node->activate();
    ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

    waitForSetup();
  }

  std::string controller_name = "test_diff_drive_controller";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot;
  std::vector<std::string> left_wheel_names;
  std::vector<std::string> right_wheel_names;
  std::vector<std::string> op_mode;

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;
};

TEST_F(TestDiffDriveController, wrong_initialization)
{
  auto uninitialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  auto diff_drive_controller =
    std::make_shared<TestableDiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(uninitialized_robot, controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::SUCCESS);

  auto unconfigured_state = diff_drive_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, unconfigured_state.id());
}

TEST_F(TestDiffDriveController, correct_initialization)
{
  auto initialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  initialized_robot->init();
  auto diff_drive_controller =
    std::make_shared<TestableDiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  auto ret = diff_drive_controller->init(initialized_robot, controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::SUCCESS);

  auto inactive_state = diff_drive_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, inactive_state.id());
  EXPECT_EQ(1.1, initialized_robot->pos1);
  EXPECT_EQ(2.2, initialized_robot->pos2);
  EXPECT_EQ(3.3, initialized_robot->pos3);
}

TEST_F(TestDiffDriveController, configuration)
{
  auto diff_drive_controller =
    std::make_shared<TestableDiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  ASSERT_EQ(
    diff_drive_controller->init(
      test_robot,
      controller_name),
    controller_interface::return_type::SUCCESS);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(diff_drive_controller->get_lifecycle_node()->get_node_base_interface());

  auto state = diff_drive_controller->get_lifecycle_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // add in check for linear vel command values
}

TEST_F(TestDiffDriveController, cleanup)
{
  auto diff_drive_controller =
    std::make_shared<TestableDiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  ASSERT_EQ(
    diff_drive_controller->init(
      test_robot,
      controller_name),
    controller_interface::return_type::SUCCESS);

  auto diff_drive_lifecycle_node = diff_drive_controller->get_lifecycle_node();
  diff_drive_lifecycle_node->set_parameter(rclcpp::Parameter("wheel_separation", 0.4));
  diff_drive_lifecycle_node->set_parameter(rclcpp::Parameter("wheel_radius", 0.1));

  auto state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  rclcpp::executors::SingleThreadedExecutor executor;
  setup_controller(*diff_drive_controller, executor);


  waitForSetup();

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);
  diff_drive_controller->wait_for_twist(executor);

  diff_drive_controller->update();
  test_robot->write();

  state = diff_drive_lifecycle_node->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  diff_drive_controller->update();
  test_robot->write();

  state = diff_drive_lifecycle_node->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  test_robot->write();

  // shouild be stopped
  EXPECT_EQ(0, test_robot->cmd1);
  EXPECT_EQ(0, test_robot->cmd2);

  executor.cancel();
}

TEST_F(TestDiffDriveController, correct_initialization_using_parameters)
{
  auto diff_drive_controller =
    std::make_shared<TestableDiffDriveController>(
    left_wheel_names,
    right_wheel_names, op_mode);
  ASSERT_EQ(
    diff_drive_controller->init(
      test_robot,
      controller_name),
    controller_interface::return_type::SUCCESS);

  // This block is replacing the way parameters are set via launch
  auto diff_drive_lifecycle_node = diff_drive_controller->get_lifecycle_node();
  rclcpp::Parameter joint_parameters("left_wheel_names", left_wheel_names);
  diff_drive_lifecycle_node->set_parameter(joint_parameters);

  std::vector<std::string> operation_mode_names =
  {test_robot->write_op_handle_name1, test_robot->write_op_handle_name2};
  rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  diff_drive_lifecycle_node->set_parameter(operation_mode_parameters);

  diff_drive_lifecycle_node->set_parameter(rclcpp::Parameter("wheel_separation", 0.4));
  diff_drive_lifecycle_node->set_parameter(rclcpp::Parameter("wheel_radius", 1.0));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(diff_drive_lifecycle_node->get_node_base_interface());

  auto state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(1.1, test_robot->cmd1);
  EXPECT_EQ(2.2, test_robot->cmd2);

  state = diff_drive_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // send msg
  const double linear = 1.0;
  const double angular = 0.0;
  publish(linear, angular);
  // wait for msg is be published to the system
  ASSERT_TRUE(diff_drive_controller->wait_for_twist(executor));

  diff_drive_controller->update();
  test_robot->write();
  EXPECT_EQ(1.0, test_robot->cmd1);
  EXPECT_EQ(1.0, test_robot->cmd2);

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = diff_drive_lifecycle_node->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  diff_drive_controller->update();
  test_robot->write();

  EXPECT_EQ(0., test_robot->cmd1) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0., test_robot->cmd2) << "Wheels are halted on deactivate()";

  // cleanup
  state = diff_drive_lifecycle_node->cleanup();
  test_robot->write();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0, test_robot->cmd1);
  EXPECT_EQ(0, test_robot->cmd2);

  state = diff_drive_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
