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

using joint_trajectory_controller::fill_partial_goal;
using joint_trajectory_controller::sort_to_local_joint_order;
using joint_trajectory_controller::validate_trajectory_msg;

std::string vectorToString(const std::vector<double> & vec)
{
  std::ostringstream oss;
  for (const auto & val : vec)
  {
    oss << val << " ";
  }
  return oss.str();
}

// Custom matcher to check if a trajectory matches the expected joint names and values
MATCHER_P2(IsTrajMsgSorted, joint_names, vector_val, "")
{
  if (arg->joint_names != joint_names)
  {
    *result_listener << "Joint names mismatch";
    return false;
  }

  if (arg->points.size() == 0)
  {
    *result_listener << "Trajectory has no points";
    return false;
  }

  const auto & point = arg->points[0];
  if (
    point.positions != vector_val || point.velocities != vector_val ||
    point.accelerations != vector_val || point.effort != vector_val)
  {
    *result_listener << "Point values mismatch:";
    if (point.positions != vector_val)
    {
      *result_listener << "\npositions: " << vectorToString(point.positions) << " ne "
                       << vectorToString(vector_val);
    }
    if (point.velocities != vector_val)
    {
      *result_listener << "\nvelocities: " << vectorToString(point.velocities) << " ne "
                       << vectorToString(vector_val);
    }
    if (point.accelerations != vector_val)
    {
      *result_listener << "\naccelerations: " << vectorToString(point.accelerations) << " ne "
                       << vectorToString(vector_val);
    }
    if (point.effort != vector_val)
    {
      *result_listener << "\neffort: " << vectorToString(point.effort) << " ne "
                       << vectorToString(vector_val);
    }
    return false;
  }

  return true;
}

template <typename T>
auto get_jumbled_values = [](const std::vector<size_t> & jumble_map, const std::vector<T> & values)
{
  std::vector<T> result;
  for (const auto & index : jumble_map)
  {
    result.push_back(values[index]);
  }
  return result;
};

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
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_TRUE(validate_trajectory_msg(good_traj_msg, *logger_, clock_.now(), params_));

  // Incompatible joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name"};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name1", "bad_name2", "bad_name3"};
  EXPECT_FALSE(validate_trajectory_msg(traj_msg, *logger_, clock_.now(), params_));

  // Empty joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names.clear();
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
  traj_msg.points[0].positions = {1.0, 2.0, 3.0};
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
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
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

/**
 * @brief test_msg_sorting test sort_to_local_joint_order()
 *
 */
TEST_F(TrajectoryOperationsTest, test_msg_sorting)
{
  std::vector<size_t> jumble_map = {1, 2, 0};
  auto vector_val = std::vector<double>{1.0, 2.0, 3.0};

  trajectory_msgs::msg::JointTrajectory good_traj_msg, traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions = vector_val;
  good_traj_msg.points[0].velocities = vector_val;
  good_traj_msg.points[0].accelerations = vector_val;
  good_traj_msg.points[0].effort = vector_val;

  // test if the joint order is not changed
  {
    auto traj_msg_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(good_traj_msg);
    EXPECT_THAT(traj_msg_ptr, IsTrajMsgSorted(joint_names_, vector_val));
    ASSERT_NO_THROW(sort_to_local_joint_order(traj_msg_ptr, *logger_, params_));
    EXPECT_THAT(traj_msg_ptr, IsTrajMsgSorted(joint_names_, vector_val));
  }

  // test if the joint order is sorted
  traj_msg = good_traj_msg;
  traj_msg.joint_names = get_jumbled_values<std::string>(jumble_map, joint_names_);
  traj_msg.points[0].positions = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg.points[0].velocities = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg.points[0].accelerations = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg.points[0].effort = get_jumbled_values<double>(jumble_map, vector_val);
  {
    auto traj_msg_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(traj_msg);
    EXPECT_THAT(traj_msg_ptr, testing::Not(IsTrajMsgSorted(joint_names_, vector_val)));
    ASSERT_NO_THROW(sort_to_local_joint_order(traj_msg_ptr, *logger_, params_));
    EXPECT_THAT(traj_msg_ptr, IsTrajMsgSorted(joint_names_, vector_val));
  }

  // test if no issue if called with different size (should not happen, because no valid_traj_msg)
  traj_msg = good_traj_msg;
  traj_msg.joint_names = get_jumbled_values<std::string>({1, 2}, joint_names_);
  ASSERT_NO_THROW(sort_to_local_joint_order(
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(traj_msg), *logger_, params_));
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.resize(1);
  traj_msg.points[0].velocities.resize(1);
  traj_msg.points[0].accelerations.resize(1);
  traj_msg.points[0].effort.resize(1);
  ASSERT_NO_THROW(sort_to_local_joint_order(
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(traj_msg), *logger_, params_));
}

TEST_F(TrajectoryOperationsTest, TestFillPartialGoal)
{
  // Create a trajectory message
  auto vector_val = std::vector<double>{1.0, 2.0, 3.0};
  trajectory_msgs::msg::JointTrajectory traj_msg_partial;
  std::vector<size_t> jumble_map = {0, 2};
  traj_msg_partial.joint_names = get_jumbled_values<std::string>(jumble_map, joint_names_);
  traj_msg_partial.header.stamp = rclcpp::Time(0);
  traj_msg_partial.points.resize(1);
  traj_msg_partial.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  traj_msg_partial.points[0].positions = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg_partial.points[0].velocities = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg_partial.points[0].accelerations = get_jumbled_values<double>(jumble_map, vector_val);
  traj_msg_partial.points[0].effort = get_jumbled_values<double>(jumble_map, vector_val);
  auto traj_msg_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(traj_msg_partial);

  // Function to return default position for a joint
  auto get_default_position = [](size_t index) { return 10. + static_cast<double>(index); };

  // Call fill_partial_goal function
  fill_partial_goal(traj_msg_ptr, get_default_position, params_);

  // Check if the fields are correctly filled (missing joints are added at the end)
  EXPECT_THAT(
    traj_msg_ptr->joint_names,
    testing::ContainerEq(get_jumbled_values<std::string>({0, 2, 1}, joint_names_)));
  ASSERT_THAT(traj_msg_ptr->points, testing::SizeIs(1));
  const auto point = traj_msg_ptr->points[0];
  ASSERT_THAT(point.positions, testing::SizeIs(params_.joints.size()));
  ASSERT_THAT(point.velocities, testing::SizeIs(params_.joints.size()));
  ASSERT_THAT(point.accelerations, testing::SizeIs(params_.joints.size()));
  ASSERT_THAT(point.effort, testing::SizeIs(params_.joints.size()));
  for (size_t i = 0; i < jumble_map.size(); ++i)
  {
    // should be original points
    EXPECT_EQ(point.positions[i], traj_msg_partial.points[0].positions[i]);
    EXPECT_EQ(point.velocities[i], traj_msg_partial.points[0].velocities[i]);
    EXPECT_EQ(point.accelerations[i], traj_msg_partial.points[0].accelerations[i]);
    EXPECT_EQ(point.effort[i], traj_msg_partial.points[0].effort[i]);
  }
  for (size_t i = jumble_map.size(); i < params_.joints.size(); ++i)
  {
    // Check if positions are set to default position, and velocities, accelerations, and efforts
    // are set to 0.0
    auto it = std::find(params_.joints.begin(), params_.joints.end(), traj_msg_ptr->joint_names[i]);
    ASSERT_NE(it, params_.joints.end()) << "Joint " << traj_msg_ptr->joint_names[i] << " not found";
    auto index = std::distance(params_.joints.begin(), it);
    EXPECT_EQ(point.positions[i], get_default_position(index));
    EXPECT_EQ(point.velocities[i], 0.0);
    EXPECT_EQ(point.accelerations[i], 0.0);
    EXPECT_EQ(point.effort[i], 0.0);
  }
}
