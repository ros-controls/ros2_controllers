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

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

class TestableDiffDriveController : public diff_drive_controller::DiffDriveController
{
public:
  using DiffDriveController::DiffDriveController;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> getLastReceivedTwist()
  {
    std::shared_ptr<geometry_msgs::msg::TwistStamped> ret;
    received_velocity_msg_ptr_.get(ret);
    return ret;
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
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(velocity_command_subscriber_);

    if (wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready)
    {
      executor.spin_some();
      return true;
    }
    return false;
  }
};

class TestDiffDriveController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableDiffDriveController>();

    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      controller_name + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  /// Publish velocity msgs
  /**
   *  linear - magnitude of the linear command in the geometry_msgs::twist message
   *  angular - the magnitude of the angular command in geometry_msgs::twist message
   */
  void publish(double linear, double angular)
  {
    int wait_count = 0;
    auto topic = velocity_publisher->get_topic_name();
    while (pub_node->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
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
    while (velocity_publisher->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL();
      }
      rclcpp::spin_some(pub_node);
    }
  }

  void assignResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(left_wheel_pos_state_);
    state_ifs.emplace_back(right_wheel_pos_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(left_wheel_vel_cmd_);
    command_ifs.emplace_back(right_wheel_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  const std::string controller_name = "test_diff_drive_controller";
  std::unique_ptr<TestableDiffDriveController> controller_;

  const std::vector<std::string> left_wheel_names = {"left_wheel_joint"};
  const std::vector<std::string> right_wheel_names = {"right_wheel_joint"};
  std::vector<double> position_values_ = {0.1, 0.2};
  std::vector<double> velocity_values_ = {0.01, 0.02};

  hardware_interface::StateInterface left_wheel_pos_state_{
    left_wheel_names[0], HW_IF_POSITION, &position_values_[0]};
  hardware_interface::StateInterface right_wheel_pos_state_{
    right_wheel_names[0], HW_IF_POSITION, &position_values_[1]};
  hardware_interface::CommandInterface left_wheel_vel_cmd_{
    left_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::CommandInterface right_wheel_vel_cmd_{
    right_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[1]};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;
};

TEST_F(TestDiffDriveController, configure_fails_without_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, configure_fails_with_only_left_or_only_right_side_defined)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(std::vector<std::string>())));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(std::vector<std::string>())));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, configure_fails_with_mismatching_wheel_side_size)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));

  auto extended_right_wheel_names = right_wheel_names;
  extended_right_wheel_names.push_back("extra_wheel");
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(extended_right_wheel_names)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, configure_succeeds_when_wheels_are_specified)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_THAT(
    controller_->state_interface_configuration().names,
    SizeIs(left_wheel_names.size() + right_wheel_names.size()));
  ASSERT_THAT(
    controller_->command_interface_configuration().names,
    SizeIs(left_wheel_names.size() + right_wheel_names.size()));
}

TEST_F(TestDiffDriveController, activate_fails_without_resources_assigned)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, activate_succeeds_with_resources_assigned)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestDiffDriveController, cleanup)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_separation", 0.4));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 0.1));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  auto state = controller_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  // should be stopped
  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value());

  executor.cancel();
}

TEST_F(TestDiffDriveController, correct_initialization_using_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_names)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_names)));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_separation", 0.4));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 1.0));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->configure();
  assignResources();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(0.01, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.02, right_wheel_vel_cmd_.get_value());

  state = controller_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // send msg
  const double linear = 1.0;
  const double angular = 0.0;
  publish(linear, angular);
  // wait for msg is be published to the system
  ASSERT_TRUE(controller_->wait_for_twist(executor));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(1.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(1.0, right_wheel_vel_cmd_.get_value());

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value());

  state = controller_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
