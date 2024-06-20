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

#ifndef TEST_TOLERANCES_UTILS_HPP_
#define TEST_TOLERANCES_UTILS_HPP_

#include <vector>

#include "gmock/gmock.h"

#include "joint_trajectory_controller/tolerances.hpp"

double default_goal_time = 0.1;
double stopped_velocity_tolerance = 0.1;

void expectDefaultTolerances(joint_trajectory_controller::SegmentTolerances active_tolerances)
{
  // acceleration is never set, and goal_state_tolerance.velocity from stopped_velocity_tolerance

  ASSERT_EQ(active_tolerances.state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).position, 0.1);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).position, 0.1);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).position, 0.1);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).acceleration, 0.0);

  ASSERT_EQ(active_tolerances.goal_state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).position, 0.1);
  EXPECT_DOUBLE_EQ(
    active_tolerances.goal_state_tolerance.at(0).velocity, stopped_velocity_tolerance);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).position, 0.1);
  EXPECT_DOUBLE_EQ(
    active_tolerances.goal_state_tolerance.at(1).velocity, stopped_velocity_tolerance);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).position, 0.1);
  EXPECT_DOUBLE_EQ(
    active_tolerances.goal_state_tolerance.at(2).velocity, stopped_velocity_tolerance);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).acceleration, 0.0);
}

#endif  // TEST_TOLERANCES_UTILS_HPP_
