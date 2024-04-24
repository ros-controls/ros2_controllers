// Copyright 2024 ros2_control Development Team
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

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "gmock/gmock.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "joint_trajectory_controller/trajectory_operations.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "test_trajectory_controller_utils.hpp"

using joint_trajectory_controller::validate_trajectory_msg;
// The fixture for testing class.
class TrajectoryOperationsTest : public testing::Test
{
protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  TrajectoryOperationsTest()
  {
    // You can do set-up work for each test here.
  }

  ~TrajectoryOperationsTest() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
    joint_names_ = {"joint1", "joint2", "joint3"};
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("test_trajectory_operations"));

    params_.joints = joint_names_;
    params_.allow_nonzero_velocity_at_trajectory_end = true;
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Class members declared here can be used by all tests in the test suite
  std::vector<std::string> joint_names_;
  rclcpp::Clock clock_;
  joint_trajectory_controller::Params params_;
  std::shared_ptr<rclcpp::Logger> logger_;
};

/**
 * @brief invalid_message Test mismatched joint and reference vector sizes
 */
TEST_F(TrajectoryOperationsTest, invalid_message)
{
  params_.allow_partial_joints_goal = false;
  params_.allow_integration_in_goal_trajectories = false;

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_TRUE(validate_trajectory_msg(good_traj_msg, *logger_, clock_.now(), params_));

  // Incompatible joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name"};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // empty message
  traj_msg = good_traj_msg;
  traj_msg.points.clear();
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0, 2.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {1.0, 2.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Effort is not supported in trajectory message
  traj_msg = good_traj_msg;
  traj_msg.points[0].effort = {1.0, 2.0, 3.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Non-strictly increasing waypoint times
  traj_msg = good_traj_msg;
  traj_msg.points.push_back(traj_msg.points.front());
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));
}

/**
 * @brief Test invalid velocity at trajectory end with parameter set to false
 */
TEST_F(
  TrajectoryOperationsTest,
  expect_invalid_when_message_with_nonzero_end_velocity_and_when_param_false)
{
  params_.allow_nonzero_velocity_at_trajectory_end = false;

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0);

  // empty message (no throw!)
  ASSERT_NO_THROW(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Nonzero velocity at trajectory end!
  traj_msg.points.resize(1);
  traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  traj_msg.points[0].positions.resize(1);
  traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  traj_msg.points[0].velocities.resize(1);
  traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));
}

/**
 * @brief missing_positions_message_accepted Test mismatched joint and reference vector sizes
 *
 * @note With allow_integration_in_goal_trajectories parameter trajectory missing position or
 * velocities are accepted
 */
TEST_F(TrajectoryOperationsTest, missing_positions_message_accepted)
{
  params_.allow_integration_in_goal_trajectories = true;

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  good_traj_msg.points[0].accelerations.resize(1);
  good_traj_msg.points[0].accelerations = {1.0, 2.0, 3.0};
  EXPECT_TRUE(validate_trajectory_msg(good_traj_msg, *logger_, clock_.now(), params_));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_TRUE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // No position and velocity data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].velocities.clear();
  EXPECT_TRUE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // All empty
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].velocities.clear();
  traj_msg.points[0].accelerations.clear();
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {2.0};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));
}
