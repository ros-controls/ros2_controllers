// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef _MSC_VER
#include <cxxabi.h>
#endif
#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <ratio>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "control_msgs/action/detail/follow_joint_trajectory__struct.hpp"
#include "controller_interface/controller_interface.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/resource_manager.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "test_trajectory_controller_utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using test_trajectory_controllers::TestableJointTrajectoryController;
using test_trajectory_controllers::TrajectoryControllerTest;
using trajectory_msgs::msg::JointTrajectoryPoint;

class TestTrajectoryActions : public TrajectoryControllerTest
{
protected:
  void SetUp()
  {
    TrajectoryControllerTest::SetUp();
    goal_options_.result_callback =
      std::bind(&TestTrajectoryActions::common_result_response, this, _1);
    goal_options_.feedback_callback = nullptr;
  }

  void SetUpExecutor(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    setup_executor_ = true;

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    SetUpAndActivateTrajectoryController(true, parameters);

    executor_->add_node(traj_node_->get_node_base_interface());

    SetUpActionClient();

    executor_->add_node(node_->get_node_base_interface());

    executor_future_handle_ = std::async(std::launch::async, [&]() -> void { executor_->spin(); });
  }

  void SetUpControllerHardware()
  {
    setup_controller_hw_ = true;

    controller_hw_thread_ = std::thread([&]() {
      // controller hardware cycle update loop
      auto start_time = rclcpp::Clock().now();
      rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
      auto end_time = start_time + wait;
      while (rclcpp::Clock().now() < end_time)
      {
        traj_controller_->update(rclcpp::Clock().now(), rclcpp::Clock().now() - start_time);
      }
    });

    // common_goal_response and common_result_response
    // sometimes doesn't receive calls when we don't sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  void SetUpActionClient()
  {
    action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_->get_node_base_interface(), node_->get_node_graph_interface(),
      node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
      controller_name_ + "/follow_joint_trajectory");

    bool response = action_client_->wait_for_action_server(std::chrono::seconds(1));
    if (!response)
    {
      throw std::runtime_error("could not get action server");
    }
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void TearDown()
  {
    TearDownControllerHardware();
    TearDownExecutor();
  }

  void TearDownExecutor()
  {
    if (setup_executor_)
    {
      setup_executor_ = false;
      executor_->cancel();
      executor_future_handle_.wait();
    }
  }

  void TearDownControllerHardware()
  {
    if (setup_controller_hw_)
    {
      setup_controller_hw_ = false;
      if (controller_hw_thread_.joinable())
      {
        controller_hw_thread_.join();
      }
    }
  }

  using FollowJointTrajectoryMsg = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>;
  using GoalOptions = rclcpp_action::Client<FollowJointTrajectoryMsg>::SendGoalOptions;

  std::shared_future<typename GoalHandle::SharedPtr> sendActionGoal(
    const std::vector<JointTrajectoryPoint> & points, double timeout, const GoalOptions & opt)
  {
    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(timeout);
    goal_msg.trajectory.joint_names = joint_names_;
    goal_msg.trajectory.points = points;

    return action_client_->async_send_goal(goal_msg, opt);
  }

  rclcpp_action::Client<FollowJointTrajectoryMsg>::SharedPtr action_client_;
  rclcpp_action::ResultCode common_resultcode_ = rclcpp_action::ResultCode::UNKNOWN;
  int common_action_result_code_ = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

  bool setup_executor_ = false;
  rclcpp::executors::MultiThreadedExecutor::UniquePtr executor_;
  std::future<void> executor_future_handle_;

  bool setup_controller_hw_ = false;
  std::thread controller_hw_thread_;

  GoalOptions goal_options_;

public:
  void common_result_response(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "common_result_response time: %f", rclcpp::Clock().now().seconds());
    common_resultcode_ = result.code;
    common_action_result_code_ = result.result->error_code;
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_DEBUG(node_->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(node_->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_DEBUG(node_->get_logger(), "Unknown result code");
        return;
    }
  }
};

TEST_F(TestTrajectoryActions, test_success_single_point_sendgoal)
{
  SetUpExecutor();
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;

    points.push_back(point);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  EXPECT_EQ(1.0, joint_pos_[0]);
  EXPECT_EQ(2.0, joint_pos_[1]);
  EXPECT_EQ(3.0, joint_pos_[2]);
}

TEST_F(TestTrajectoryActions, test_success_multi_point_sendgoal)
{
  SetUpExecutor();
  SetUpControllerHardware();

  // add feedback
  bool feedback_recv = false;
  goal_options_.feedback_callback =
    [&](
      rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectoryMsg::Feedback>) { feedback_recv = true; };

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal with multiple points
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names_.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names_.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  EXPECT_NEAR(7.0, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(8.0, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(9.0, joint_pos_[2], COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_goal_tolerances_single_point_success)
{
  // set tolerance parameters
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.goal", 0.1),
    rclcpp::Parameter("constraints.joint2.goal", 0.1),
    rclcpp::Parameter("constraints.joint3.goal", 0.1)};

  SetUpExecutor(params);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL, common_action_result_code_);

  EXPECT_NEAR(1.0, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(2.0, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(3.0, joint_pos_[2], COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_goal_tolerances_multi_point_success)
{
  // set tolerance parameters
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.goal", 0.1),
    rclcpp::Parameter("constraints.joint2.goal", 0.1),
    rclcpp::Parameter("constraints.joint3.goal", 0.1)};

  SetUpExecutor(params);
  SetUpControllerHardware();

  // add feedback
  bool feedback_recv = false;
  goal_options_.feedback_callback =
    [&](
      rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectoryMsg::Feedback>) { feedback_recv = true; };

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal with multiple points
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names_.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names_.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL, common_action_result_code_);

  EXPECT_NEAR(7.0, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(8.0, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(9.0, joint_pos_[2], COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_state_tolerances_fail)
{
  // set joint tolerance parameters
  const double state_tol = 0.0001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint2.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint3.trajectory", state_tol)};

  SetUpExecutor(params);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 4.0;
    point.positions[1] = 5.0;
    point.positions[2] = 6.0;
    points.push_back(point);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED,
    common_action_result_code_);
}

TEST_F(TestTrajectoryActions, test_cancel_hold_position)
{
  SetUpExecutor();
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 4.0;
    point.positions[1] = 5.0;
    point.positions[2] = 6.0;
    points.push_back(point);

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
    goal_msg.trajectory.joint_names = joint_names_;
    goal_msg.trajectory.points = points;

    // send and wait for half a second before cancel
    gh_future = action_client_->async_send_goal(goal_msg, goal_options_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const auto goal_handle = gh_future.get();
    action_client_->async_cancel_goal(goal_handle);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL, common_action_result_code_);

  const double prev_pos1 = joint_pos_[0];
  const double prev_pos2 = joint_pos_[1];
  const double prev_pos3 = joint_pos_[2];

  // run an update, it should be holding
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  EXPECT_EQ(prev_pos1, joint_pos_[0]);
  EXPECT_EQ(prev_pos2, joint_pos_[1]);
  EXPECT_EQ(prev_pos3, joint_pos_[2]);
}
