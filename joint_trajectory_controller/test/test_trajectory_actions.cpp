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

#include <cxxabi.h>
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

#include "gtest/gtest.h"
#include "action_msgs/msg/goal_status_array.hpp"
#include "control_msgs/action/detail/follow_joint_trajectory__struct.hpp"
#include "controller_interface/controller_interface.hpp"
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
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using trajectory_msgs::msg::JointTrajectoryPoint;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
const double COMMON_THRESHOLD = 0.001;
}

class TestTrajectoryActions : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
    test_robot_->init();
    joint_names_ = {{test_robot_->joint_name1, test_robot_->joint_name2, test_robot_->joint_name3}};
    op_mode_ = {{test_robot_->write_op_handle_name1}};

    node_ = std::make_shared<rclcpp::Node>("trajectory_test_node");

    traj_controller_ = std::make_shared<joint_trajectory_controller::JointTrajectoryController>();
    auto ret = traj_controller_->init(test_robot_, controller_name_);
    if (ret != controller_interface::return_type::SUCCESS) {
      FAIL();
    }

    traj_lifecycle_node_ = traj_controller_->get_lifecycle_node();
    rclcpp::Parameter joint_parameters("joints", joint_names_);
    traj_lifecycle_node_->set_parameter(joint_parameters);

    rclcpp::Parameter operation_mode_parameters("write_op_modes", op_mode_);
    traj_lifecycle_node_->set_parameter(operation_mode_parameters);

    // ignore velocity tolerances for this test since they arent commited in test_robot->write()
    rclcpp::Parameter stopped_velocity_parameters("constraints.stopped_velocity_tolerance", 0.0);
    traj_lifecycle_node_->set_parameter(stopped_velocity_parameters);

    goal_options_.goal_response_callback =
      std::bind(&TestTrajectoryActions::common_goal_response, this, _1);
    goal_options_.result_callback =
      std::bind(&TestTrajectoryActions::common_result_response, this, _1);
    goal_options_.feedback_callback = nullptr;
  }

  void SetUpExecutor()
  {
    setup_executor_ = true;

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    executor_->add_node(traj_lifecycle_node_->get_node_base_interface());

    traj_controller_->on_configure(traj_lifecycle_node_->get_current_state());
    traj_controller_->on_activate(traj_lifecycle_node_->get_current_state());

    SetUpActionClient();

    executor_->add_node(node_->get_node_base_interface());

    executor_future_handle_ = std::async(
      std::launch::async, [&]() -> void {
        executor_->spin();
      });
  }

  void SetUpControllerHardware()
  {
    setup_controller_hw_ = true;

    controller_hw_thread_ = std::thread(
      [&]() {
        // controller hardware cycle update loop
        auto start_time = rclcpp::Clock().now();
        rclcpp::Duration wait = rclcpp::Duration::from_seconds(2.0);
        auto end_time = start_time + wait;
        while (rclcpp::Clock().now() < end_time) {
          test_robot_->read();
          traj_controller_->update();
          test_robot_->write();
        }
      });

    // common_goal_response and common_result_response
    // sometimes doesnt receive calls when we dont sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  void SetUpActionClient()
  {
    action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      controller_name_ + "/follow_joint_trajectory");

    bool response =
      action_client_->wait_for_action_server(std::chrono::seconds(1));
    if (!response) {
      throw std::runtime_error("could not get action server");
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void TearDown()
  {
    TearDownControllerHardware();
    TearDownExecutor();
  }

  void TearDownExecutor()
  {
    if (setup_executor_) {
      setup_executor_ = false;
      executor_->cancel();
      executor_future_handle_.wait();
    }
  }

  void TearDownControllerHardware()
  {
    if (setup_controller_hw_) {
      setup_controller_hw_ = false;
      if (controller_hw_thread_.joinable()) {
        controller_hw_thread_.join();
      }
    }
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
    goal_msg.trajectory.joint_names = joint_names_;
    goal_msg.trajectory.points = points;

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, opt);
    return true;
  }

  std::string controller_name_ = "test_joint_trajectory_actions";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> op_mode_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<joint_trajectory_controller::JointTrajectoryController> traj_controller_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr traj_lifecycle_node_;

  rclcpp_action::Client<FollowJointTrajectoryMsg>::SharedPtr action_client_;
  rclcpp_action::ResultCode common_resultcode_ = rclcpp_action::ResultCode::UNKNOWN;
  bool common_goal_accepted_ = false;
  int common_action_result_code_ = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

  bool setup_executor_ = false;
  rclcpp::executors::MultiThreadedExecutor::UniquePtr executor_;
  std::future<void> executor_future_handle_;

  bool setup_controller_hw_ = false;
  std::thread controller_hw_thread_;

  GoalOptions goal_options_;

public:
  void common_goal_response(std::shared_future<GoalHandle::SharedPtr> future)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "common_goal_response time: %f",
      rclcpp::Clock().now().seconds());
    const auto goal_handle = future.get();
    if (!goal_handle) {
      common_goal_accepted_ = false;
      RCLCPP_DEBUG(node_->get_logger(), "Goal rejected");
    } else {
      common_goal_accepted_ = true;
      RCLCPP_DEBUG(node_->get_logger(), "Goal accepted");
    }
  }

  void common_result_response(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "common_result_response time: %f",
      rclcpp::Clock().now().seconds());
    common_resultcode_ = result.code;
    common_action_result_code_ = result.result->error_code;
    switch (result.code) {
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

TEST_F(TestTrajectoryActions, test_success_single_point_sendgoal) {
  SetUpExecutor();
  SetUpControllerHardware();

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

    sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  EXPECT_EQ(1.0, test_robot_->pos1);
  EXPECT_EQ(2.0, test_robot_->pos2);
  EXPECT_EQ(3.0, test_robot_->pos3);
}

TEST_F(TestTrajectoryActions, test_success_multi_point_sendgoal) {
  SetUpExecutor();
  SetUpControllerHardware();

  // add feedback
  bool feedback_recv = false;
  goal_options_.feedback_callback = [&](
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

    sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);

  EXPECT_NEAR(7.0, test_robot_->pos1, COMMON_THRESHOLD);
  EXPECT_NEAR(8.0, test_robot_->pos2, COMMON_THRESHOLD);
  EXPECT_NEAR(9.0, test_robot_->pos3, COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_goal_tolerances_single_point_success) {
  // set tolerance parameters
  traj_lifecycle_node_->declare_parameter("constraints.joint1.goal", 0.1);
  traj_lifecycle_node_->declare_parameter("constraints.joint2.goal", 0.1);
  traj_lifecycle_node_->declare_parameter("constraints.joint3.goal", 0.1);

  SetUpExecutor();
  SetUpControllerHardware();

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

    sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code_);

  EXPECT_NEAR(1.0, test_robot_->pos1, COMMON_THRESHOLD);
  EXPECT_NEAR(2.0, test_robot_->pos2, COMMON_THRESHOLD);
  EXPECT_NEAR(3.0, test_robot_->pos3, COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_goal_tolerances_multi_point_success) {
  // set tolerance parameters
  traj_lifecycle_node_->declare_parameter("constraints.joint1.goal", 0.1);
  traj_lifecycle_node_->declare_parameter("constraints.joint2.goal", 0.1);
  traj_lifecycle_node_->declare_parameter("constraints.joint3.goal", 0.1);

  SetUpExecutor();
  SetUpControllerHardware();

  // add feedback
  bool feedback_recv = false;
  goal_options_.feedback_callback = [&](
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

    sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(feedback_recv);
  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code_);

  EXPECT_NEAR(7.0, test_robot_->pos1, COMMON_THRESHOLD);
  EXPECT_NEAR(8.0, test_robot_->pos2, COMMON_THRESHOLD);
  EXPECT_NEAR(9.0, test_robot_->pos3, COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryActions, test_state_tolerances_fail) {
  // set joint tolerance parameters
  const double state_tol = 0.0001;
  traj_lifecycle_node_->declare_parameter("constraints.joint1.trajectory", state_tol);
  traj_lifecycle_node_->declare_parameter("constraints.joint2.trajectory", state_tol);
  traj_lifecycle_node_->declare_parameter("constraints.joint3.trajectory", state_tol);

  SetUpExecutor();
  SetUpControllerHardware();

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

    sendActionGoal(points, 1.0, goal_options_);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED,
    common_action_result_code_);
}

TEST_F(TestTrajectoryActions, test_cancel_hold_position) {
  SetUpExecutor();
  SetUpControllerHardware();

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
    const auto goal_handle_future = action_client_->async_send_goal(goal_msg, goal_options_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const auto goal_handle = goal_handle_future.get();
    action_client_->async_cancel_goal(goal_handle);
  }
  controller_hw_thread_.join();

  EXPECT_TRUE(common_goal_accepted_);
  EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, common_resultcode_);
  EXPECT_EQ(
    control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
    common_action_result_code_);

  const double prev_pos1 = test_robot_->pos1;
  const double prev_pos2 = test_robot_->pos2;
  const double prev_pos3 = test_robot_->pos3;

  // run an update, it should be holding
  traj_controller_->update();
  test_robot_->write();

  EXPECT_EQ(prev_pos1, test_robot_->pos1);
  EXPECT_EQ(prev_pos2, test_robot_->pos2);
  EXPECT_EQ(prev_pos3, test_robot_->pos3);
}
