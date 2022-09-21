// Copyright 2022 Pixel Robotics.
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

/*
 * Author: Tony Najjar
 */

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tricycle_controller/tricycle_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

class TestableTricycleController : public tricycle_controller::TricycleController
{
public:
  using TricycleController::TricycleController;
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

class TestTricycleController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableTricycleController>();
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
    state_ifs.emplace_back(steering_joint_pos_state_);
    state_ifs.emplace_back(traction_joint_vel_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(steering_joint_pos_cmd_);
    command_ifs.emplace_back(traction_joint_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  const std::string controller_name = "test_tricycle_controller";
  std::unique_ptr<TestableTricycleController> controller_;

  const std::string traction_joint_name = "traction_joint";
  const std::string steering_joint_name = "steering_joint";
  double position_ = 0.1;
  double velocity_ = 0.2;

  hardware_interface::StateInterface steering_joint_pos_state_{
    steering_joint_name, HW_IF_POSITION, &position_};

  hardware_interface::StateInterface traction_joint_vel_state_{
    traction_joint_name, HW_IF_VELOCITY, &velocity_};

  hardware_interface::CommandInterface steering_joint_pos_cmd_{
    steering_joint_name, HW_IF_POSITION, &position_};

  hardware_interface::CommandInterface traction_joint_vel_cmd_{
    traction_joint_name, HW_IF_VELOCITY, &velocity_};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;
};

TEST_F(TestTricycleController, configure_fails_without_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestTricycleController, configure_fails_if_only_traction_or_steering_side_defined)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(std::string())));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(std::string())));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestTricycleController, configure_succeeds_when_joints_are_specified)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestTricycleController, activate_fails_without_resources_assigned)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestTricycleController, activate_succeeds_with_resources_assigned)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  // We implicitly test that by default position feedback is required
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestTricycleController, cleanup)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheelbase", 1.2));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 0.12));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->get_node()->activate();
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

  state = controller_->get_node()->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  // should be stopped
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value());

  executor.cancel();
}

TEST_F(TestTricycleController, correct_initialization_using_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::OK);

  controller_->get_node()->set_parameter(
    rclcpp::Parameter("traction_joint_name", rclcpp::ParameterValue(traction_joint_name)));
  controller_->get_node()->set_parameter(
    rclcpp::Parameter("steering_joint_name", rclcpp::ParameterValue(steering_joint_name)));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheelbase", 0.4));
  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 1.0));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  assignResources();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(position_, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(velocity_, traction_joint_vel_cmd_.get_value());

  state = controller_->get_node()->activate();
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
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(1.0, traction_joint_vel_cmd_.get_value());

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, steering_joint_pos_cmd_.get_value());
  EXPECT_EQ(0.0, traction_joint_vel_cmd_.get_value());

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
