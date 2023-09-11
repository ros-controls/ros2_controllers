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

  void SetUpExecutor(
    const std::vector<rclcpp::Parameter> & parameters = {},
    bool separate_cmd_and_state_values = false)
  {
    setup_executor_ = true;

    SetUpAndActivateTrajectoryController(executor_, parameters, separate_cmd_and_state_values);

    SetUpActionClient();

    executor_.add_node(node_->get_node_base_interface());

    executor_future_handle_ = std::async(std::launch::async, [&]() -> void { executor_.spin(); });
  }

  void SetUpControllerHardware()
  {
    setup_controller_hw_ = true;

    controller_hw_thread_ = std::thread(
      [&]()
      {
        // controller hardware cycle update loop
        auto clock = rclcpp::Clock(RCL_STEADY_TIME);
        auto start_time = clock.now();
        rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
        auto end_time = start_time + wait;
        while (clock.now() < end_time)
        {
          traj_controller_->update(clock.now(), clock.now() - start_time);
        }
      });

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
      executor_.cancel();
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
  rclcpp::executors::MultiThreadedExecutor executor_;
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

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class TestTrajectoryActionsTestParameterized
: public TestTrajectoryActions,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::string>, std::vector<std::string>>>
{
public:
  virtual void SetUp()
  {
    TestTrajectoryActions::SetUp();
    command_interface_types_ = std::get<0>(GetParam());
    state_interface_types_ = std::get<1>(GetParam());
  }

  static void TearDownTestCase() { TrajectoryControllerTest::TearDownTestCase(); }
};

TEST_P(TestTrajectoryActionsTestParameterized, test_success_single_point_sendgoal)
{
  SetUpExecutor();
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  std::vector<double> point_positions{1.0, 2.0, 3.0};
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions = point_positions;

    points.push_back(point);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the last position goal
  // i.e., active but trivial trajectory (one point only)
  // note: the action goal also is a trivial trajectory
  expectHoldingPoint(point_positions);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_success_multi_point_sendgoal)
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
  std::vector<std::vector<double>> points_positions{{{4.0, 5.0, 6.0}}, {{7.0, 8.0, 9.0}}};
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names_.size());

    point1.positions = points_positions.at(0);
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names_.size());

    point2.positions = points_positions.at(1);
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the last position goal
  // i.e., active but trivial trajectory (one point only)
  expectHoldingPoint(points_positions.at(1));
}

/**
 * Makes sense with position command interface only,
 * because no integration to position state interface is implemented
 */
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
  std::vector<std::vector<double>> points_positions{{{1.0, 2.0, 3.0}}};
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions = points_positions.at(0);
    points.push_back(point);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL, common_action_result_code_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the last position goal
  // i.e., active but trivial trajectory (one point only)
  expectHoldingPoint(points_positions.at(0));
}

/**
 * Makes sense with position command interface only,
 * because no integration to position state interface is implemented
 */
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
  std::vector<std::vector<double>> points_positions{{{4.0, 5.0, 6.0}}, {{7.0, 8.0, 9.0}}};
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names_.size());

    point1.positions = points_positions.at(0);
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names_.size());

    point2.positions = points_positions.at(1);
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL, common_action_result_code_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the last position goal
  // i.e., active but trivial trajectory (one point only)
  expectHoldingPoint(points_positions.at(1));
}

TEST_P(TestTrajectoryActionsTestParameterized, test_state_tolerances_fail)
{
  // set joint tolerance parameters
  const double state_tol = 0.0001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint2.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint3.trajectory", state_tol)};

  // separate command from states -> immediate state tolerance fail
  bool separate_cmd_and_state_values = true;
  SetUpExecutor(params, separate_cmd_and_state_values);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  std::vector<std::vector<double>> points_positions{{{4.0, 5.0, 6.0}}, {{7.0, 8.0, 9.0}}};
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point1.positions.resize(joint_names_.size());

    point1.positions = points_positions.at(0);
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.1);
    point2.positions.resize(joint_names_.size());

    point2.positions = points_positions.at(1);
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED,
    common_action_result_code_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the position (being the initial one)
  // i.e., active but trivial trajectory (one point only)
  std::vector<double> initial_positions{INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
  expectHoldingPoint(initial_positions);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_goal_tolerances_fail)
{
  // set joint tolerance parameters
  const double goal_tol = 0.1;
  // set very small goal_time so that goal_time is violated
  const double goal_time = 0.000001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.goal", goal_tol),
    rclcpp::Parameter("constraints.joint2.goal", goal_tol),
    rclcpp::Parameter("constraints.joint3.goal", goal_tol),
    rclcpp::Parameter("constraints.goal_time", goal_time)};

  // separate command from states -> goal won't never be reached
  bool separate_cmd_and_state_values = true;
  SetUpExecutor(params, separate_cmd_and_state_values);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal; one point only -> command is directly set to reach this goal (no interpolation)
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);
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
    control_msgs::action::FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED,
    common_action_result_code_);

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the position (being the initial one)
  // i.e., active but trivial trajectory (one point only)
  std::vector<double> initial_positions{INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
  expectHoldingPoint(initial_positions);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_no_time_from_start_state_tolerance_fail)
{
  // set joint tolerance parameters
  const double state_tol = 0.0001;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("constraints.joint1.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint2.trajectory", state_tol),
    rclcpp::Parameter("constraints.joint3.trajectory", state_tol)};

  // separate command from states -> goal won't never be reached
  bool separate_cmd_and_state_values = true;
  SetUpExecutor(params, separate_cmd_and_state_values);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);
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

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the position (being the initial one)
  // i.e., active but trivial trajectory (one point only)
  std::vector<double> initial_positions{INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
  expectHoldingPoint(initial_positions);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_cancel_hold_position)
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

  std::vector<double> cancelled_position{joint_pos_[0], joint_pos_[1], joint_pos_[2]};

  // run an update
  updateController(rclcpp::Duration::from_seconds(0.01));

  // it should be holding the last position,
  // i.e., active but trivial trajectory (one point only)
  expectHoldingPoint(cancelled_position);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_allow_nonzero_velocity_at_trajectory_end_true)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", true)};
  SetUpExecutor(params);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal with nonzero last velocities
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point1.positions.resize(joint_names_.size());
    point1.velocities.resize(joint_names_.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    point1.velocities[0] = 4.0;
    point1.velocities[1] = 5.0;
    point1.velocities[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.1);
    point2.positions.resize(joint_names_.size());
    point2.velocities.resize(joint_names_.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    point2.velocities[0] = 4.0;
    point2.velocities[1] = 5.0;
    point2.velocities[2] = 6.0;
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  // will be accepted despite nonzero last point
  EXPECT_TRUE(gh_future.get());
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
}

TEST_P(TestTrajectoryActionsTestParameterized, test_allow_nonzero_velocity_at_trajectory_end_false)
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", false)};
  SetUpExecutor(params);
  SetUpControllerHardware();

  std::shared_future<typename GoalHandle::SharedPtr> gh_future;
  // send goal with nonzero last velocities
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point1.positions.resize(joint_names_.size());
    point1.velocities.resize(joint_names_.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    point1.velocities[0] = 4.0;
    point1.velocities[1] = 5.0;
    point1.velocities[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.1);
    point2.positions.resize(joint_names_.size());
    point2.velocities.resize(joint_names_.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    point2.velocities[0] = 4.0;
    point2.velocities[1] = 5.0;
    point2.velocities[2] = 6.0;
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_FALSE(gh_future.get());

  // send goal with last velocity being zero
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point1.positions.resize(joint_names_.size());
    point1.velocities.resize(joint_names_.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    point1.velocities[0] = 4.0;
    point1.velocities[1] = 5.0;
    point1.velocities[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.1);
    point2.positions.resize(joint_names_.size());
    point2.velocities.resize(joint_names_.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    point2.velocities[0] = 0.0;
    point2.velocities[1] = 0.0;
    point2.velocities[2] = 0.0;
    points.push_back(point2);

    gh_future = sendActionGoal(points, 1.0, goal_options_);
  }

  EXPECT_TRUE(gh_future.get());
}

// position controllers
INSTANTIATE_TEST_SUITE_P(
  PositionTrajectoryControllersActions, TestTrajectoryActionsTestParameterized,
  ::testing::Values(
    std::make_tuple(std::vector<std::string>({"position"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// position_velocity controllers
INSTANTIATE_TEST_SUITE_P(
  PositionVelocityTrajectoryControllersActions, TestTrajectoryActionsTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// only velocity controller
INSTANTIATE_TEST_SUITE_P(
  OnlyVelocityTrajectoryControllersAction, TestTrajectoryActionsTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"velocity"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"velocity"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// only effort controller
INSTANTIATE_TEST_SUITE_P(
  OnlyEffortTrajectoryControllers, TestTrajectoryActionsTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"effort"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"effort"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));
