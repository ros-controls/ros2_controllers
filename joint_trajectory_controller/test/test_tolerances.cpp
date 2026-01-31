// Copyright 2024 Austrian Institute of Technology
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

#include <gmock/gmock.h>

#include <cmath>
#include <vector>

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "joint_trajectory_controller/tolerances.hpp"
#include "test_trajectory_controller_utils.hpp"

using joint_trajectory_controller::SegmentTolerances;
using trajectory_msgs::msg::JointTrajectoryPoint;

std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};

control_msgs::action::FollowJointTrajectory_Goal prepareGoalMsg(
  const std::vector<JointTrajectoryPoint> & points, double goal_time_tolerance,
  const std::vector<control_msgs::msg::JointTolerance> path_tolerance =
    std::vector<control_msgs::msg::JointTolerance>(),
  const std::vector<control_msgs::msg::JointTolerance> goal_tolerance =
    std::vector<control_msgs::msg::JointTolerance>())
{
  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(goal_time_tolerance);
  goal_msg.goal_tolerance = goal_tolerance;
  goal_msg.path_tolerance = path_tolerance;
  goal_msg.trajectory.joint_names = joint_names_;
  goal_msg.trajectory.points = points;

  return goal_msg;
}
class TestTolerancesFixture : public ::testing::Test
{
protected:
  SegmentTolerances default_tolerances;
  joint_trajectory_controller::Params params;
  std::vector<std::string> joint_names_;
  rclcpp::Logger logger = rclcpp::get_logger("TestTolerancesFixture");

  void SetUp() override
  {
    // Initialize joint_names_ with some test data
    joint_names_ = {"joint1", "joint2", "joint3"};

    // Initialize default_tolerances and params with common setup for all tests
    // TODO(anyone) fill params and use
    //    SegmentTolerances get_segment_tolerances(Params const & params) instead
    default_tolerances.goal_time_tolerance = default_goal_time;
    default_tolerances.state_tolerance.resize(joint_names_.size());
    default_tolerances.goal_state_tolerance.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      default_tolerances.state_tolerance.at(i).position = 0.1;
      default_tolerances.goal_state_tolerance.at(i).position = 0.1;
      default_tolerances.goal_state_tolerance.at(i).velocity = stopped_velocity_tolerance;
    }
    params.joints = joint_names_;
  }

  void TearDown() override
  {
    // Cleanup code if necessary
  }
};

TEST_F(TestTolerancesFixture, test_get_segment_tolerances)
{
  // send goal with nonzero tolerances, are they accepted?
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  // add the same tolerance for every joint, give it in correct order
  tolerance.name = "joint1";
  tolerance.position = 0.2;
  tolerance.velocity = 0.3;
  tolerance.acceleration = 0.4;
  path_tolerance.push_back(tolerance);
  tolerance.name = "joint2";
  path_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  path_tolerance.push_back(tolerance);
  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  // add different tolerances in jumbled order
  tolerance.name = "joint2";
  tolerance.position = 1.2;
  tolerance.velocity = 2.2;
  tolerance.acceleration = 3.2;
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  tolerance.position = 1.3;
  tolerance.velocity = 2.3;
  tolerance.acceleration = 3.3;
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint1";
  tolerance.position = 1.1;
  tolerance.velocity = 2.1;
  tolerance.acceleration = 3.1;
  goal_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, 2.0, path_tolerance, goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);

  EXPECT_DOUBLE_EQ(active_tolerances.goal_time_tolerance, 2.0);

  ASSERT_EQ(active_tolerances.state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).acceleration, 0.4);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).acceleration, 0.4);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).acceleration, 0.4);

  ASSERT_EQ(active_tolerances.goal_state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).position, 1.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).velocity, 2.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).acceleration, 3.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).position, 1.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).velocity, 2.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).acceleration, 3.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).position, 1.3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).velocity, 2.3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).acceleration, 3.3);
}

// send goal with deactivated tolerances (-1)
TEST_F(TestTolerancesFixture, test_deactivate_tolerances)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  // add the same tolerance for every joint, give it in correct order
  tolerance.name = "joint1";
  tolerance.position = -1.0;
  tolerance.velocity = -1.0;
  tolerance.acceleration = -1.0;
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint2";
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, -1.0, path_tolerance, goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);

  EXPECT_DOUBLE_EQ(active_tolerances.goal_time_tolerance, 0.0);

  ASSERT_EQ(active_tolerances.state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).acceleration, 0.0);

  ASSERT_EQ(active_tolerances.goal_state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).acceleration, 0.0);
}

// send goal with invalid tolerances, are the default ones used?
TEST_F(TestTolerancesFixture, test_invalid_tolerances)
{
  {
    SCOPED_TRACE("negative goal_time_tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, -123.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path position tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = -123.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path velocity tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = -123.0;
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path acceleration tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = -123.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal position tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = -123.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal velocity tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = -123.0;
    tolerance.acceleration = 0.0;
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal acceleration tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = -123.0;
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
}
TEST_F(TestTolerancesFixture, test_invalid_joints_path_tolerance)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = "joint123";
  path_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);
  expectDefaultTolerances(active_tolerances);
}
TEST_F(TestTolerancesFixture, test_invalid_joints_goal_tolerance)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = "joint123";
  goal_tolerance.push_back(tolerance);

  auto goal_msg =
    prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);
  expectDefaultTolerances(active_tolerances);
}

// Create Error Point Tests
TEST_F(TestTolerancesFixture, test_create_error_trajectory_point)
{
  trajectory_msgs::msg::JointTrajectoryPoint desired;
  trajectory_msgs::msg::JointTrajectoryPoint actual;

  // Setup desired state
  desired.positions = {1.0, 2.0, 3.0};
  desired.velocities = {0.1, 0.2, 0.3};
  desired.accelerations = {0.01, 0.02, 0.03};

  // Setup actual state
  actual.positions = {0.9, 2.1, 3.0};
  actual.velocities = {0.05, 0.25, 0.3};
  actual.accelerations = {0.0, 0.03, 0.03};

  // Calculate error: Error = Desired - Actual
  auto error_point = joint_trajectory_controller::create_error_trajectory_point(desired, actual);

  // Verify Position Errors
  ASSERT_EQ(error_point.positions.size(), 3);
  EXPECT_NEAR(error_point.positions[0], 0.1, 1e-6);
  EXPECT_NEAR(error_point.positions[1], -0.1, 1e-6);
  EXPECT_NEAR(error_point.positions[2], 0.0, 1e-6);

  // Verify Velocity Errors
  ASSERT_EQ(error_point.velocities.size(), 3);
  EXPECT_NEAR(error_point.velocities[0], 0.05, 1e-6);
  EXPECT_NEAR(error_point.velocities[1], -0.05, 1e-6);
  EXPECT_NEAR(error_point.velocities[2], 0.0, 1e-6);

  // Verify Acceleration Errors
  ASSERT_EQ(error_point.accelerations.size(), 3);
  EXPECT_NEAR(error_point.accelerations[0], 0.01, 1e-6);
  EXPECT_NEAR(error_point.accelerations[1], -0.01, 1e-6);
  EXPECT_NEAR(error_point.accelerations[2], 0.0, 1e-6);
}

TEST_F(TestTolerancesFixture, test_create_error_trajectory_point_mismatched_sizes)
{
  trajectory_msgs::msg::JointTrajectoryPoint desired;
  trajectory_msgs::msg::JointTrajectoryPoint actual;

  desired.positions = {1.0, 2.0};
  actual.positions = {1.0, 2.0, 3.0};  // Mismatched size

  // The function should return an empty error state if sizes don't match
  auto error_point = joint_trajectory_controller::create_error_trajectory_point(desired, actual);

  EXPECT_TRUE(error_point.positions.empty());
  EXPECT_TRUE(error_point.velocities.empty());
  EXPECT_TRUE(error_point.accelerations.empty());
}
