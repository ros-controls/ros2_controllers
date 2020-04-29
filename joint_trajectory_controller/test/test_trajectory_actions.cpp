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

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "controller_parameter_server/parameter_server.hpp"

#include "hardware_interface/robot_hardware.hpp"

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_robot_hardware/test_robot_hardware.hpp"

using trajectory_msgs::msg::JointTrajectoryPoint;
using std::placeholders::_1;
using std::placeholders::_2;

class TestTrajectoryActions : public ::testing::Test
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
    joint_names = {{test_robot->joint_name1, test_robot->joint_name2, test_robot->joint_name3}};
    op_mode = {{test_robot->write_op_handle_name1}};

    node = std::make_shared<rclcpp::Node>("trajectory_test_node");

    traj_controller = std::make_shared<joint_trajectory_controller::JointTrajectoryController>();
    auto ret = traj_controller->init(test_robot, controller_name);
    if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      FAIL();
    }

    traj_lifecycle_node = traj_controller->get_lifecycle_node();
    rclcpp::Parameter joint_parameters("joints", joint_names);
    traj_lifecycle_node->set_parameter(joint_parameters);

    rclcpp::Parameter operation_mode_parameters("write_op_modes", op_mode);
    traj_lifecycle_node->set_parameter(operation_mode_parameters);
  }

  void SetUpActionClient()
  {
    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      controller_name + "/follow_joint_trajectory");

    bool response =
      action_client->wait_for_action_server(std::chrono::seconds(1));
    if (!response) {
      throw std::runtime_error("could not get action server");
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
  using FollowJointTrajectoryMsg = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>;
  using GoalOptions = rclcpp_action::Client<FollowJointTrajectoryMsg>::SendGoalOptions;

  bool sendActionGoal(
    const std::vector<JointTrajectoryPoint> & points,
    double timeout,
    const GoalOptions & opt)
  {
    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(timeout);
    goal_msg.trajectory.joint_names = joint_names;
    goal_msg.trajectory.points = points;

    auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);
    return true;
  }

  std::string controller_name = "test_joint_trajectory_actions";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot;
  std::vector<std::string> joint_names;
  std::vector<std::string> op_mode;

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<joint_trajectory_controller::JointTrajectoryController> traj_controller;
  rclcpp_lifecycle::LifecycleNode::SharedPtr traj_lifecycle_node;

  rclcpp_action::Client<FollowJointTrajectoryMsg>::SharedPtr action_client;
  rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
  bool common_goal_accepted = false;
  int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
  double common_threshold = 0.001;

public:
  void common_goal_response(std::shared_future<GoalHandle::SharedPtr> future)
  {
    RCLCPP_DEBUG(
      node->get_logger(), "common_goal_response time: %f",
      rclcpp::Clock().now().seconds());
    auto goal_handle = future.get();
    if (!goal_handle) {
      common_goal_accepted = false;
      RCLCPP_DEBUG(node->get_logger(), "Goal rejected");
    } else {
      common_goal_accepted = true;
      RCLCPP_DEBUG(node->get_logger(), "Goal accepted");
    }
  }

  void common_result_response(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_DEBUG(
      node->get_logger(), "common_result_response time: %f",
      rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_DEBUG(node->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(node->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_DEBUG(node->get_logger(), "Unknown result code");
        return;
    }
  }
};

TEST_F(TestTrajectoryActions, test_success_multi_point_sendgoal) {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());
  SetUpActionClient();
  executor.add_node(node->get_node_base_interface());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  auto thread_func = [&]() {
      // controller hardware cycle update loop
      auto start_time = rclcpp::Clock().now();
      rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
      auto end_time = start_time + wait;
      while (rclcpp::Clock().now() < end_time) {
        test_robot->read();
        traj_controller->update();
        test_robot->write();
      }
    };
  std::thread controller_hw_thread(thread_func);

  // common_goal_response and common_result_response
  // sometimes doesnt receive calls when we dont sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  GoalOptions opt;
  opt.goal_response_callback =
    std::bind(&TestTrajectoryActions::common_goal_response, this, _1);
  opt.result_callback =
    std::bind(&TestTrajectoryActions::common_result_response, this, _1);
  opt.feedback_callback = nullptr;

  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
    point.positions.resize(joint_names.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;

    points.push_back(point);

    sendActionGoal(points, 1.0, opt);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode);

  EXPECT_EQ(1.0, test_robot->pos1);
  EXPECT_EQ(2.0, test_robot->pos2);
  EXPECT_EQ(3.0, test_robot->pos3);

  // start again
  common_goal_accepted = false;
  common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
  controller_hw_thread = std::thread(thread_func);

  // add feedback
  bool feedback_recv = false;
  opt.feedback_callback = [&](
    rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectoryMsg::Feedback> feedback)
    {
      (void)feedback;
      feedback_recv = true;
    };

  // send goal with multiple points
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    points.push_back(point2);

    sendActionGoal(points, 1.0, opt);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, feedback_recv);
  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode);

  EXPECT_NEAR(7.0, test_robot->pos1, common_threshold);
  EXPECT_NEAR(8.0, test_robot->pos2, common_threshold);
  EXPECT_NEAR(9.0, test_robot->pos3, common_threshold);

  executor.cancel();
}

TEST_F(TestTrajectoryActions, test_goal_tolerances_success) {
  // set tolerance parameters
  traj_lifecycle_node->declare_parameter("constraints.joint1.goal", 0.0);
  traj_lifecycle_node->declare_parameter("constraints.joint2.goal", 0.0);
  traj_lifecycle_node->declare_parameter("constraints.joint3.goal", 0.0);
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint1.goal", 0.1));
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint2.goal", 0.1));
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint3.goal", 0.1));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());
  SetUpActionClient();
  executor.add_node(node->get_node_base_interface());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  auto thread_func = [&]() {
      // controller hardware cycle update loop
      auto start_time = rclcpp::Clock().now();
      rclcpp::Duration wait = rclcpp::Duration::from_seconds(3.0);
      auto end_time = start_time + wait;
      while (rclcpp::Clock().now() < end_time) {
        test_robot->read();
        traj_controller->update();
        test_robot->write();
      }
    };
  std::thread controller_hw_thread(thread_func);

  // common_goal_response and common_result_response
  // sometimes doesnt receive calls when we dont sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  GoalOptions opt;
  opt.goal_response_callback =
    std::bind(&TestTrajectoryActions::common_goal_response, this, _1);
  opt.result_callback =
    std::bind(&TestTrajectoryActions::common_result_response, this, _1);
  opt.feedback_callback = nullptr;

  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
    point.positions.resize(joint_names.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    sendActionGoal(points, 1.0, opt);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code);

  EXPECT_NEAR(1.0, test_robot->pos1, common_threshold);
  EXPECT_NEAR(2.0, test_robot->pos2, common_threshold);
  EXPECT_NEAR(3.0, test_robot->pos3, common_threshold);

  // start again
  common_goal_accepted = false;
  common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
  controller_hw_thread = std::thread(thread_func);

  // add feedback
  bool feedback_recv = false;
  opt.feedback_callback = [&](
    rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectoryMsg::Feedback> feedback)
    {
      (void)feedback;
      feedback_recv = true;
    };

  // send goal with multiple points
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.2);
    point1.positions.resize(joint_names.size());

    point1.positions[0] = 4.0;
    point1.positions[1] = 5.0;
    point1.positions[2] = 6.0;
    points.push_back(point1);

    JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(0.3);
    point2.positions.resize(joint_names.size());

    point2.positions[0] = 7.0;
    point2.positions[1] = 8.0;
    point2.positions[2] = 9.0;
    points.push_back(point2);

    sendActionGoal(points, 1.0, opt);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, feedback_recv);
  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code);

  EXPECT_NEAR(7.0, test_robot->pos1, common_threshold);
  EXPECT_NEAR(8.0, test_robot->pos2, common_threshold);
  EXPECT_NEAR(9.0, test_robot->pos3, common_threshold);

  executor.cancel();
}

TEST_F(TestTrajectoryActions, test_state_tolerances_fail) {
  // set joint tolerance parameters
  double state_tol = 0.0001;
  traj_lifecycle_node->declare_parameter("constraints.joint1.trajectory", 0.0);
  traj_lifecycle_node->declare_parameter("constraints.joint2.trajectory", 0.0);
  traj_lifecycle_node->declare_parameter("constraints.joint3.trajectory", 0.0);
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint1.trajectory", state_tol));
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint2.trajectory", state_tol));
  traj_lifecycle_node->set_parameter(rclcpp::Parameter("constraints.joint3.trajectory", state_tol));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());
  SetUpActionClient();
  executor.add_node(node->get_node_base_interface());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  auto thread_func = [&]() {
      // controller hardware cycle update loop
      auto start_time = rclcpp::Clock().now();
      rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
      auto end_time = start_time + wait;
      while (rclcpp::Clock().now() < end_time) {
        test_robot->read();
        traj_controller->update();
        test_robot->write();
      }
    };
  std::thread controller_hw_thread(thread_func);

  // common_goal_response and common_result_response
  // sometimes doesnt receive calls when we dont sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  GoalOptions opt;
  opt.goal_response_callback =
    std::bind(&TestTrajectoryActions::common_goal_response, this, _1);
  opt.result_callback =
    std::bind(&TestTrajectoryActions::common_result_response, this, _1);
  opt.feedback_callback = nullptr;

  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    point.positions.resize(joint_names.size());

    point.positions[0] = 4.0;
    point.positions[1] = 5.0;
    point.positions[2] = 6.0;
    points.push_back(point);

    sendActionGoal(points, 1.0, opt);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, common_resultcode);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED,
    common_action_result_code);

  executor.cancel();
}

TEST_F(TestTrajectoryActions, test_cancel_hold_position) {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());
  SetUpActionClient();
  executor.add_node(node->get_node_base_interface());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  auto thread_func = [&]() {
      auto start_time = rclcpp::Clock().now();
      rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
      auto end_time = start_time + wait;
      while (rclcpp::Clock().now() < end_time) {
        test_robot->read();
        traj_controller->update();
        test_robot->write();
      }
    };
  std::thread controller_hw_thread(thread_func);

  // common_goal_response and common_result_response
  // sometimes doesnt receive calls when we dont sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  GoalOptions opt;
  opt.goal_response_callback =
    std::bind(&TestTrajectoryActions::common_goal_response, this, _1);
  opt.result_callback =
    std::bind(&TestTrajectoryActions::common_result_response, this, _1);

  // send goal
  {
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    point.positions.resize(joint_names.size());

    point.positions[0] = 4.0;
    point.positions[1] = 5.0;
    point.positions[2] = 6.0;
    points.push_back(point);

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
    goal_msg.trajectory.joint_names = joint_names;
    goal_msg.trajectory.points = points;

    // send and wait for half a second before cancel
    auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto goal_handle = goal_handle_future.get();
    action_client->async_cancel_goal(goal_handle);
  }
  controller_hw_thread.join();

  EXPECT_EQ(true, common_goal_accepted);
  EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, common_resultcode);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code);

  double prev_pos1 = test_robot->pos1;
  double prev_pos2 = test_robot->pos2;
  double prev_pos3 = test_robot->pos3;

  // run an update, it should be holding
  traj_controller->update();
  test_robot->write();

  EXPECT_EQ(prev_pos1, test_robot->pos1);
  EXPECT_EQ(prev_pos2, test_robot->pos2);
  EXPECT_EQ(prev_pos3, test_robot->pos3);

  executor.cancel();
}
