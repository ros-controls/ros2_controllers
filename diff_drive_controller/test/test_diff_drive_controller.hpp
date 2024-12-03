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

#ifndef TEST_DIFF_DRIVE_CONTROLLER_HPP_
#define TEST_DIFF_DRIVE_CONTROLLER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

namespace
{
const std::vector<std::string> left_wheel_names = {"left_wheel_joint"};
const std::vector<std::string> right_wheel_names = {"right_wheel_joint"};
}  // namespace

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

  /**
   * @brief Used to get the real_time_odometry_publisher to verify its contents
   *
   * @return Copy of realtime_odometry_publisher_ object
   */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  get_rt_odom_publisher()
  {
    return realtime_odometry_publisher_;
  }
};

class TestDiffDriveController : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // use the name of the test as the controller name (i.e, the node name) to be able to set
    // parameters from yaml for each test
    controller_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
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

  void assignResourcesPosFeedback()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(left_wheel_pos_state_);
    state_ifs.emplace_back(right_wheel_pos_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(left_wheel_vel_cmd_);
    command_ifs.emplace_back(right_wheel_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void assignResourcesVelFeedback()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(left_wheel_vel_state_);
    state_ifs.emplace_back(right_wheel_vel_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(left_wheel_vel_cmd_);
    command_ifs.emplace_back(right_wheel_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(
    const std::vector<std::string> left_wheel_joints_init = left_wheel_names,
    const std::vector<std::string> right_wheel_joints_init = right_wheel_names,
    const std::vector<rclcpp::Parameter> & parameters = {}, const std::string ns = "")
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    parameter_overrides.push_back(
      rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_joints_init)));
    parameter_overrides.push_back(
      rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_joints_init)));
    // default parameters
    parameter_overrides.push_back(
      rclcpp::Parameter("wheel_separation", rclcpp::ParameterValue(1.0)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.1)));

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    return controller_->init(controller_name, urdf_, 0, ns, node_options);
  }

  std::string controller_name;
  std::unique_ptr<TestableDiffDriveController> controller_;

  std::vector<double> position_values_ = {0.1, 0.2};
  std::vector<double> velocity_values_ = {0.01, 0.02};

  hardware_interface::StateInterface left_wheel_pos_state_{
    left_wheel_names[0], HW_IF_POSITION, &position_values_[0]};
  hardware_interface::StateInterface right_wheel_pos_state_{
    right_wheel_names[0], HW_IF_POSITION, &position_values_[1]};
  hardware_interface::StateInterface left_wheel_vel_state_{
    left_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::StateInterface right_wheel_vel_state_{
    right_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[1]};
  hardware_interface::CommandInterface left_wheel_vel_cmd_{
    left_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::CommandInterface right_wheel_vel_cmd_{
    right_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[1]};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;

  const std::string urdf_ = "";
};

#endif  // TEST_DIFF_DRIVE_CONTROLLER_HPP_
