// Copyright 2023 AIT Austrian Institute of Technology
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

#ifndef TEST_PID_TRAJECTORY_HPP_
#define TEST_PID_TRAJECTORY_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gmock/gmock.h"

#include "joint_trajectory_controller_plugins/pid_trajectory_plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_pid_trajectory");
}  // namespace

class TestableJointTrajectoryControllerPlugin
: public joint_trajectory_controller_plugins::PidTrajectoryPlugin
{
public:
  // updates mapped parameters, i.e., should be called after setting the joint names
  void trigger_declare_parameters() { param_listener_->declare_params(); }
};

class PidTrajectoryTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    auto testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(testname);
    executor_->add_node(node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  PidTrajectoryTest() { executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(); }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    node_.reset();
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

#endif  // TEST_PID_TRAJECTORY_HPP_
