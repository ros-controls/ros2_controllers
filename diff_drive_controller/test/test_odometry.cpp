// Copyright 2026 Vedhas Talnikar
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

#include "diff_drive_controller/odometry.hpp"
#include "gmock/gmock.h"

class OdometryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize odometry with a default velocity rolling window size
    odometry_.setVelocityRollingWindowSize(10);

    // Wheel separation = 1.0m
    // This simplifies the math: 1 rad/s wheel vel = 1 m/s linear vel
    odometry_.setWheelParams(1.0, 1.0, 1.0);
  }

  diff_drive_controller::Odometry odometry_;
};

TEST_F(OdometryTest, TestInitialState)
{
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinear(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getAngular(), 0.0);
}

TEST_F(OdometryTest, TestLinearMotion)
{
  // Move forward: Left = 1.0 rad/s, Right = 1.0 rad/s, dt = 1.0s
  // Expected Linear Vel = (1.0 + 1.0) / 2 = 1.0 m/s
  // Expected Angular Vel = (1.0 - 1.0) / 1.0 = 0.0 rad/s
  // This triggers the linear approximation path in integrate()
  bool result = odometry_.update_from_vel(1.0, 1.0, 1.0);

  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(odometry_.getX(), 1.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinear(), 1.0);
  EXPECT_DOUBLE_EQ(odometry_.getAngular(), 0.0);
}

TEST_F(OdometryTest, TestPureRotation)
{
  // Rotate in place: Left = -1.0, Right = 1.0, dt = 1.0s
  // Expected Linear Vel = (-1.0 + 1.0) / 2 = 0.0 m/s
  // Expected Angular Vel = (1.0 - (-1.0)) / 1.0 = 2.0 rad/s
  // This triggers the exact integration path in integrate()
  bool result = odometry_.update_from_vel(-1.0, 1.0, 1.0);
  EXPECT_TRUE(result);

  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 2.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinear(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getAngular(), 2.0);
}

TEST_F(OdometryTest, TestCurvedMotion_ExactArc)
{
  // Curve: Left = 1.0, Right = 2.0, dt = 1.0s
  // Linear = (1+2)/2 = 1.5 m/s
  // Angular = (2-1)/1 = 1.0 rad/s
  // This triggers the "else" block in integrate() for exact arc calculation

  bool result = odometry_.update_from_vel(1.0, 2.0, 1.0);
  EXPECT_TRUE(result);

  // heading_new = 0 + 1.0 * 1.0 = 1.0 rad
  // x = (v/w) * (sin(heading_new) - sin(0)) = (1.5/1.0) * (sin(1.0))
  // y = -(v/w) * (cos(heading_new) - cos(0)) = -1.5 * (cos(1.0) - 1)

  const double expected_x = 1.5 * std::sin(1.0);
  const double expected_y = -1.5 * (std::cos(1.0) - 1.0);

  EXPECT_NEAR(odometry_.getX(), expected_x, 1e-5);
  EXPECT_NEAR(odometry_.getY(), expected_y, 1e-5);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 1.0);
}

TEST_F(OdometryTest, TestSmallDtRejection)
{
  // Provide a dt smaller than the 1e-6 threshold in the code (it checks < 1e-6)
  bool result = odometry_.update_from_vel(1.0, 1.0, 1e-7);

  EXPECT_FALSE(result);
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
}

TEST_F(OdometryTest, TestOpenLoopUpdate)
{
  // Directly feed v=2.0, w=0.5, dt=1.0
  bool result = odometry_.try_update_open_loop(2.0, 0.5, 1.0);

  EXPECT_TRUE(result);

  // heading = 0.5 rad
  // x = (2.0/0.5) * sin(0.5) = 4 * sin(0.5)
  // y = -4 * (cos(0.5) - 1)

  const double expected_x = 4.0 * std::sin(0.5);
  const double expected_y = -4.0 * (std::cos(0.5) - 1.0);

  EXPECT_NEAR(odometry_.getX(), expected_x, 1e-5);
  EXPECT_NEAR(odometry_.getY(), expected_y, 1e-5);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.5);

  // Verify internal state matches inputs
  EXPECT_DOUBLE_EQ(odometry_.getLinear(), 2.0);
  EXPECT_DOUBLE_EQ(odometry_.getAngular(), 0.5);
}

TEST_F(OdometryTest, TestUpdateFromPosition)
{
  // Left moves 0 -> 1.0, Right moves 0 -> 1.0 over 1.0s
  // Implies vel = 1.0 for both

  bool result = odometry_.update_from_pos(1.0, 1.0, 1.0);

  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(odometry_.getX(), 1.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinear(), 1.0);
}

TEST_F(OdometryTest, TestReset)
{
  // 1. Move the robot somewhere
  bool result = odometry_.update_from_vel(1.0, 1.0, 1.0);
  EXPECT_TRUE(result);
  EXPECT_NE(odometry_.getX(), 0.0);

  // 2. Reset
  odometry_.setOdometry(0.0, 0.0, 0.0);

  // 3. Verify position is cleared
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.0);
}
