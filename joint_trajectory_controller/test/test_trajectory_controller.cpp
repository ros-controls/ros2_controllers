// Copyright 2017 Open Source Robotics Foundation, Inc.
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


#include <stddef.h>

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "control_msgs/msg/detail/joint_trajectory_controller_state__struct.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"
#include "test_trajectory_controller_utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TestableJointTrajectoryController;
using test_trajectory_controllers::TestTrajectoryController;

void
spin(rclcpp::executors::MultiThreadedExecutor * exe)
{
  exe->spin();
}

TEST_F(TestTrajectoryController, configuration) {
  SetUpTrajectoryController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_controller_->get_node()->get_node_base_interface());
  const auto future_handle_ = std::async(std::launch::async, spin, &executor);

  const auto state = traj_controller_->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  traj_controller_->update();

  // no change in hw position
  EXPECT_NE(3.3, joint_pos_[0]);
  EXPECT_NE(4.4, joint_pos_[1]);
  EXPECT_NE(5.5, joint_pos_[2]);

  executor.cancel();
}

// TEST_F(TestTrajectoryController, activation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::OK) {
//     FAIL();
//   }
//
//   auto traj_node = traj_controller->get_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_node->get_node_base_interface());
//
//   auto state = traj_controller_->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//
//   // wait for the subscriber and publisher to completely setup
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//
//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
//   publish(time_from_start, points);
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   executor.spin_once();
//
//   traj_controller->update();
//   resource_manager_->write();
//
//   // change in hw position
//   EXPECT_EQ(3.3, joint_pos_[0]);
//   EXPECT_EQ(4.4, joint_pos_[1]);
//   EXPECT_EQ(5.5, joint_pos_[2]);
//
//   executor.cancel();
// }

// TEST_F(TestTrajectoryController, reactivation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::OK) {
//     FAIL();
//   }
//
//   auto traj_node = traj_controller->get_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_node->get_node_base_interface());
//
//   auto state = traj_controller_->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//
//   // wait for the subscriber and publisher to completely setup
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//
//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   // *INDENT-OFF*
//   std::vector<std::vector<double>> points {
//     {{3.3, 4.4, 5.5}},
//     {{7.7, 8.8, 9.9}},
//     {{10.10, 11.11, 12.12}}
//   };
//   // *INDENT-ON*
//   publish(time_from_start, points);
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   executor.spin_once();
//
//   traj_controller->update();
//   resource_manager_->write();
//
//   // deactivated
//   // wait so controller process the second point when deactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   state = traj_controller_->deactivate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//   resource_manager_->read();
//   traj_controller->update();
//   resource_manager_->write();
//
//   // no change in hw position
//   EXPECT_EQ(3.3, joint_pos_[0]);
//   EXPECT_EQ(4.4, joint_pos_[1]);
//   EXPECT_EQ(5.5, joint_pos_[2]);
//
//   // reactivated
//   // wait so controller process the third point when reactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//   resource_manager_->read();
//   traj_controller->update();
//   resource_manager_->write();
//
//   // change in hw position to 3rd point
//   EXPECT_EQ(10.10, joint_pos_[0]);
//   EXPECT_EQ(11.11, joint_pos_[1]);
//   EXPECT_EQ(12.12, joint_pos_[2]);
//
//   executor.cancel();
// }

TEST_F(TestTrajectoryController, cleanup) {
  SetUpAndActivateTrajectoryController();

  auto traj_node = traj_controller_->get_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_node->get_node_base_interface());

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  traj_controller_->wait_for_trajectory(executor);
  traj_controller_->update();

  auto state = traj_controller_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  traj_controller_->update();

  state = traj_controller_->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  // update for 0.25 seconds
  const auto start_time = rclcpp::Clock().now();
  updateController(rclcpp::Duration::from_seconds(0.25));

  // should be home pose again
  EXPECT_NEAR(INITIAL_POS_JOINT1, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(INITIAL_POS_JOINT2, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(INITIAL_POS_JOINT3, joint_pos_[2], COMMON_THRESHOLD);
}

TEST_F(TestTrajectoryController, correct_initialization_using_parameters) {
  SetUpTrajectoryController(false);

  // This block is replacing the way parameters are set via launch
  auto traj_node = traj_controller_->get_node();
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  const rclcpp::Parameter joint_parameters("joints", joint_names_);
  traj_node->set_parameter(joint_parameters);
  traj_controller_->configure();
  auto state = traj_controller_->get_current_state();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  ActivateTrajectoryController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_node->get_node_base_interface());

  state = traj_controller_->get_current_state();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());
  EXPECT_EQ(INITIAL_POS_JOINT1, joint_pos_[0]);
  EXPECT_EQ(INITIAL_POS_JOINT2, joint_pos_[1]);
  EXPECT_EQ(INITIAL_POS_JOINT3, joint_pos_[2]);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points {
    {{3.3, 4.4, 5.5}},
    {{7.7, 8.8, 9.9}},
    {{10.10, 11.11, 12.12}}
  };
  // *INDENT-ON*
  publish(time_from_start, points);
  traj_controller_->wait_for_trajectory(executor);

  // first update
  traj_controller_->update();

  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(FIRST_POINT_TIME);
  traj_controller_->update();
  // deactivated
  state = traj_controller_->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  const auto allowed_delta = 0.05;

  EXPECT_NEAR(3.3, joint_pos_[0], allowed_delta);
  EXPECT_NEAR(4.4, joint_pos_[1], allowed_delta);
  EXPECT_NEAR(5.5, joint_pos_[2], allowed_delta);

  // cleanup
  state = traj_controller_->cleanup();

  // update loop receives a new msg and updates accordingly
  traj_controller_->update();

  // check the traj_msg_home_ptr_ initialization code for the standard wait timing
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  traj_controller_->update();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  EXPECT_NEAR(INITIAL_POS_JOINT1, joint_pos_[0], allowed_delta);
  EXPECT_NEAR(INITIAL_POS_JOINT2, joint_pos_[1], allowed_delta);
  EXPECT_NEAR(INITIAL_POS_JOINT3, joint_pos_[2], allowed_delta);

  state = traj_controller_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

TEST_F(TestTrajectoryController, state_topic_consistency) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();
  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();

  size_t n_joints = joint_names_.size();

  for (unsigned int i = 0; i < n_joints; ++i) {
    EXPECT_EQ(joint_names_[i], state->joint_names[i]);
  }

  // No trajectory by default, no desired state or error
  EXPECT_TRUE(state->desired.positions.empty());
  EXPECT_TRUE(state->desired.velocities.empty());
  EXPECT_TRUE(state->desired.accelerations.empty());

  EXPECT_EQ(n_joints, state->actual.positions.size());
  EXPECT_EQ(n_joints, state->actual.velocities.size());
  EXPECT_TRUE(state->actual.accelerations.empty());

  EXPECT_TRUE(state->error.positions.empty());
  EXPECT_TRUE(state->error.velocities.empty());
  EXPECT_TRUE(state->error.accelerations.empty());
}

void TestTrajectoryController::test_state_publish_rate_target(int target_msg_count)
{
  rclcpp::Parameter state_publish_rate_param(
    "state_publish_rate",
    static_cast<double>(target_msg_count));
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {state_publish_rate_param}, &executor);

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  using control_msgs::msg::JointTrajectoryControllerState;

  const int qos_level = 10;
  int echo_received_counter = 0;
  rclcpp::Subscription<JointTrajectoryControllerState>::SharedPtr subs =
    traj_node_->create_subscription<JointTrajectoryControllerState>(
    "/state",
    qos_level,
    [&](JointTrajectoryControllerState::UniquePtr) {
      ++echo_received_counter;
    }
    );

  // update for 1second
  const auto start_time = rclcpp::Clock().now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(1.0);
  const auto end_time = start_time + wait;
  while (rclcpp::Clock().now() < end_time) {
    traj_controller_->update();
  }

  // We may miss the last message since time allowed is exactly the time needed
  EXPECT_NEAR(target_msg_count, echo_received_counter, 1);

  executor.cancel();
}

/**
 * @brief test_state_publish_rate Test that state publish rate matches configure rate
 */
TEST_F(TestTrajectoryController, test_state_publish_rate) {
  test_state_publish_rate_target(10);
}

TEST_F(TestTrajectoryController, zero_state_publish_rate) {
  test_state_publish_rate_target(0);
}

/**
 * @brief test_jumbled_joint_order Test sending trajectories with a joint order different from internal controller order
 */
TEST_F(TestTrajectoryController, test_jumbled_joint_order) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    const std::vector<std::string> jumbled_joint_names {
      joint_names_[1], joint_names_[2], joint_names_[0]
    };
    traj_msg.joint_names = jumbled_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(3);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 3.0;
    traj_msg.points[0].positions[2] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.25 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  EXPECT_NEAR(1.0, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(2.0, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(3.0, joint_pos_[2], COMMON_THRESHOLD);
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled joints
 */
TEST_F(TestTrajectoryController, test_partial_joint_list) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  const double initial_joint3_cmd = joint_pos_[2];
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names {
      joint_names_[1], joint_names_[0]
    };
    traj_msg.joint_names = partial_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(2);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 1.0;
    traj_msg.points[0].velocities.resize(2);
    traj_msg.points[0].velocities[0] = 2.0;
    traj_msg.points[0].velocities[1] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.5 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  double threshold = 0.001;
  EXPECT_NEAR(traj_msg.points[0].positions[1], joint_pos_[0], threshold);
  EXPECT_NEAR(traj_msg.points[0].positions[0], joint_pos_[1], threshold);
  EXPECT_NEAR(
    initial_joint3_cmd, joint_pos_[2],
    threshold) << "Joint 3 command should be current position";

//  Velocity commands are not sent yet
//  EXPECT_NEAR(traj_msg.points[0].velocities[1], test_robot_->vel1, threshold);
//  EXPECT_NEAR(traj_msg.points[0].velocities[0], test_robot_->vel2, threshold);
//  EXPECT_NEAR(
//    0.0, test_robot_->vel3,
//    threshold) << "Joint 3 velocity should be 0.0 since it's not in the goal";

  executor.cancel();
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled joints without allow_partial_joints_goal
 */
TEST_F(TestTrajectoryController, test_partial_joint_list_not_allowed) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  const double initial_joint1_cmd = joint_pos_[0];
  const double initial_joint2_cmd = joint_pos_[1];
  const double initial_joint3_cmd = joint_pos_[2];
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names {
      joint_names_[1], joint_names_[0]
    };
    traj_msg.joint_names = partial_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(2);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 1.0;
    traj_msg.points[0].velocities.resize(2);
    traj_msg.points[0].velocities[0] = 2.0;
    traj_msg.points[0].velocities[1] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.5 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  double threshold = 0.001;
  EXPECT_NEAR(
    initial_joint1_cmd, joint_pos_[0],
    threshold) << "All joints command should be current position because goal was rejected";
  EXPECT_NEAR(
    initial_joint2_cmd, joint_pos_[1],
    threshold) << "All joints command should be current position because goal was rejected";
  EXPECT_NEAR(
    initial_joint3_cmd, joint_pos_[2],
    threshold) << "All joints command should be current position because goal was rejected";

  //  Velocity commands are not sent yet
  //  EXPECT_NEAR(traj_msg.points[0].velocities[1], test_robot_->vel1, threshold);
  //  EXPECT_NEAR(traj_msg.points[0].velocities[0], test_robot_->vel2, threshold);
  //  EXPECT_NEAR(
  //    0.0, test_robot_->vel3,
  //    threshold) << "Joint 3 velocity should be 0.0 since it's not in the goal";

  executor.cancel();
}


/**
 * @brief invalid_message Test mismatched joint and reference vector sizes
 */
TEST_F(TestTrajectoryController, invalid_message) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // Incompatible joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name"};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few efforts
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].effort = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Non-strictly increasing waypoint times
  traj_msg = good_traj_msg;
  traj_msg.points.push_back(traj_msg.points.front());
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief test_trajectory_replace Test replacing an existing trajectory
 */
TEST_F(TestTrajectoryController, test_trajectory_replace) {
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}}};
  std::vector<std::vector<double>> points_partial_new {{1.5}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired.positions = {points_old[0].begin(), points_old[0].end()};
  //  Check that we reached end of points_old trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_node_->get_logger(), "Sending new trajectory");
  publish(time_from_start, points_partial_new);
  // Replaced trajectory is a mix of previous and current goal
  expected_desired.positions[0] = points_partial_new[0][0];
  expected_desired.positions[1] = points_old[0][1];
  expected_desired.positions[2] = points_old[0][2];
  expected_actual = expected_desired;
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}

/**
 * @brief test_ignore_old_trajectory Sending an old trajectory replacing an existing trajectory
 */
TEST_F(TestTrajectoryController, test_ignore_old_trajectory) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new {{{-1., -2., -3.}}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0] trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_node_->get_logger(), "Sending new trajectory in the past");
  //  New trajectory will end before current time
  rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
  expected_actual.positions = {points_old[1].begin(), points_old[1].end()};
  expected_desired = expected_actual;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}

TEST_F(TestTrajectoryController, test_ignore_partial_old_trajectory) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new {{{-1., -2., -3.}, {-2., -4., -6.}}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_node_->get_logger(), "Sending new trajectory partially in the past");
  //  New trajectory first point is in the past, second is in the future
  rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
  expected_actual.positions = {points_new[1].begin(), points_new[1].end()};
  expected_desired = expected_actual;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}


TEST_F(TestTrajectoryController, test_execute_partial_traj_in_future) {
  SetUpTrajectoryController();
  auto traj_node = traj_controller_->get_node();
  RCLCPP_WARN(
    traj_node->get_logger(),
    "Test disabled until current_trajectory is taken into account when adding a new trajectory.");
  // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/joint_trajectory_controller/include/joint_trajectory_controller/init_joint_trajectory.h#L149
  return;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(traj_node->get_node_base_interface());
  subscribeToState();
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  traj_node->set_parameter(partial_joints_parameters);
  traj_controller_->configure();
  traj_controller_->activate();

  std::vector<std::vector<double>> full_traj {{{2., 3., 4.}, {4., 6., 8.}}};
  std::vector<std::vector<double>> partial_traj {{{-1., -2.}, {-2., -4, }}};
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration points_delay{rclcpp::Duration(delay)};
  // Send full trajectory
  publish(points_delay, full_traj);
  // Sleep until first waypoint of full trajectory

  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {full_traj[0].begin(), full_traj[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory and are starting points_old[1]
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);


  // Send partial trajectory starting after full trajecotry is complete
  RCLCPP_INFO(traj_node->get_logger(), "Sending new trajectory in the future");
  publish(points_delay, partial_traj, rclcpp::Clock().now() + delay * 2);
  // Wait until the end start and end of partial traj

  expected_actual.positions = {partial_traj.back()[0], partial_traj.back()[1], full_traj.back()[2]};
  expected_desired = expected_actual;

  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(
      delay * (2 + 2)), 0.1);
}
