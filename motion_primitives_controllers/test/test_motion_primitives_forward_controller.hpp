// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#ifndef TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
#define TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_

#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motion_primitives_controllers/motion_primitives_forward_controller.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "control_msgs/action/execute_motion_primitive_sequence.hpp"
#include "control_msgs/msg/motion_primitive.hpp"

using MotionPrimitive = control_msgs::msg::MotionPrimitive;
using ExecuteMotion = control_msgs::action::ExecuteMotionPrimitiveSequence;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableMotionPrimitivesForwardController
: public motion_primitives_controllers::MotionPrimitivesForwardController
{
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, activate_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, reactivate_success);
  FRIEND_TEST(MotionPrimitivesForwardControllerTest, receive_single_action_goal);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return motion_primitives_controllers::MotionPrimitivesForwardController::on_configure(
      previous_state);
  }
};
// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class MotionPrimitivesForwardControllerFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    controller_ = std::make_unique<CtrlType>();
    ASSERT_EQ(
      controller_->init(
        "test_motion_primitives_forward_controller", "", 0, "",
        controller_->define_custom_node_options()),
      controller_interface::return_type::OK);

    node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(controller_->get_node()->get_node_base_interface());

    executor_thread_ = std::thread([this]() { executor_->spin(); });

    action_client_ = rclcpp_action::create_client<ExecuteMotion>(
      node_->get_node_base_interface(), node_->get_node_graph_interface(),
      node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
      "/test_motion_primitives_forward_controller/motion_sequence");
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    controller_.reset();
    node_.reset();
    executor_.reset();
  }

protected:
  void SetUpController()
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    command_itfs_.clear();
    command_itfs_.reserve(command_values_.size());
    command_ifs.reserve(command_values_.size());

    for (size_t i = 0; i < command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          interface_namespace_, command_interface_names_[i], &command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    state_itfs_.clear();
    state_itfs_.reserve(state_values_.size());
    state_ifs.reserve(state_values_.size());

    for (size_t i = 0; i < state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          interface_namespace_, state_interface_names_[i], &state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void send_single_motion_sequence_goal(
    const std::vector<double> & joint_positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
    double velocity = 0.7, double acceleration = 1.0, double move_time = 2.0,
    double blend_radius = 3.0)
  {
    std::cout << "Send motion sequence goal..." << std::endl;

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      throw std::runtime_error("Action server not available");
    }

    auto goal_msg = ExecuteMotion::Goal();
    MotionPrimitive primitive;
    primitive.type = static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT);
    primitive.joint_positions = joint_positions;
    primitive.blend_radius = blend_radius;

    primitive.additional_arguments.resize(3);
    primitive.additional_arguments[0].name = "velocity";
    primitive.additional_arguments[0].value = velocity;
    primitive.additional_arguments[1].name = "acceleration";
    primitive.additional_arguments[1].value = acceleration;
    primitive.additional_arguments[2].name = "move_time";
    primitive.additional_arguments[2].value = move_time;

    goal_msg.trajectory.motions.push_back(primitive);

    auto goal_future = action_client_->async_send_goal(goal_msg);

    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      throw std::runtime_error("Failed to send goal (future timeout)");
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      throw std::runtime_error("Goal was rejected by the action server");
    }

    std::cout << "Goal accepted by the action server." << std::endl;
  }

  std::vector<std::string> command_interface_names_ = {
    "motion_type", "q1",           "q2",         "q3",           "q4",
    "q5",          "q6",           "pos_x",      "pos_y",        "pos_z",
    "pos_qx",      "pos_qy",       "pos_qz",     "pos_qw",       "pos_via_x",
    "pos_via_y",   "pos_via_z",    "pos_via_qx", "pos_via_qy",   "pos_via_qz",
    "pos_via_qw",  "blend_radius", "velocity",   "acceleration", "move_time"};

  std::vector<std::string> state_interface_names_ = {"execution_status", "ready_for_new_primitive"};

  std::string interface_namespace_ = "motion_primitive";
  std::array<double, 2> state_values_ = {
    {static_cast<uint8_t>(motion_primitives_controllers::ExecutionState::IDLE),
     static_cast<uint8_t>(motion_primitives_controllers::ReadyForNewPrimitive::READY)}};
  std::array<double, 25> command_values_ = {
    {101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101,
     101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101,
     101.101, 101.101, 101.101, 101.101, 101.101, 101.101, 101.101}};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  std::unique_ptr<TestableMotionPrimitivesForwardController> controller_;
  rclcpp_action::Client<ExecuteMotion>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

#endif  // TEST_MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
