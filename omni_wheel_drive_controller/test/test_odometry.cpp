// Copyright 2026 Devdoot Chatterjee
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

#include "gmock/gmock.h"
#include "omni_wheel_drive_controller/odometry.hpp"

class OmniOdometryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Standard 4-wheel omni robot
    // Robot radius = 1.0m, Wheel radius = 0.1m, Wheel offset = 0.0 rads, 4 wheels
    odometry_.setParams(1.0, 0.1, 0.0, 4);
  }

  omni_wheel_drive_controller::Odometry odometry_;
};

TEST_F(OmniOdometryTest, TestInitialState)
{
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinearXVel(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinearYVel(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getAngularVel(), 0.0);
}

TEST_F(OmniOdometryTest, TestLinearMotionX)
{
  // Move purely forward in X: Vx = 1.0, Vy = 0.0, W = 0.0, dt = 1.0
  // W0 = 0, W1 = 10, W2 = 0, W3 = -10
  std::vector<double> wheel_vels = {0.0, 10.0, 0.0, -10.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1.0);

  EXPECT_TRUE(result);
  EXPECT_NEAR(odometry_.getX(), 1.0, 1e-5);
  EXPECT_NEAR(odometry_.getY(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getHeading(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getLinearXVel(), 1.0, 1e-5);
  EXPECT_NEAR(odometry_.getLinearYVel(), 0.0, 1e-5);
}

TEST_F(OmniOdometryTest, TestLinearMotionY)
{
  // Strafing purely sideways in Y: Vx = 0.0, Vy = 1.0, W = 0.0, dt = 1.0
  // W0 = -10, W1 = 0, W2 = 10, W3 = 0
  std::vector<double> wheel_vels = {-10.0, 0.0, 10.0, 0.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1.0);

  EXPECT_TRUE(result);
  EXPECT_NEAR(odometry_.getX(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getY(), 1.0, 1e-5);
  EXPECT_NEAR(odometry_.getHeading(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getLinearYVel(), 1.0, 1e-5);
}

TEST_F(OmniOdometryTest, TestPureRotation)
{
  // Rotate in place: Vx = 0.0, Vy = 0.0, W = 2.0, dt = 1.0
  // W0 = -20, W1 = -20, W2 = -20, W3 = -20
  std::vector<double> wheel_vels = {-20.0, -20.0, -20.0, -20.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1.0);

  EXPECT_TRUE(result);
  EXPECT_NEAR(odometry_.getX(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getY(), 0.0, 1e-5);
  EXPECT_NEAR(odometry_.getHeading(), 2.0, 1e-5);
  EXPECT_NEAR(odometry_.getAngularVel(), 2.0, 1e-5);
}

TEST_F(OmniOdometryTest, TestCurvedMotion_ExactArc)
{
  // Curve: Moving in X, Y, and Rotating simultaneously
  // Vx = 1.5, Vy = 0.5, W = 1.0, dt = 1.0
  // W0 = (-0.5 - 1.0)*10 = -15
  // W1 = (1.5 - 1.0)*10 = 5
  // W2 = (0.5 - 1.0)*10 = -5
  // W3 = (-1.5 - 1.0)*10 = -25
  std::vector<double> wheel_vels = {-15.0, 5.0, -5.0, -25.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1.0);
  EXPECT_TRUE(result);

  const double expected_x = (1.5 / 1.0) * std::sin(1.0) + (0.5 / 1.0) * (std::cos(1.0) - 1.0);
  const double expected_y = -(1.5 / 1.0) * (std::cos(1.0) - 1.0) + (0.5 / 1.0) * std::sin(1.0);

  EXPECT_NEAR(odometry_.getX(), expected_x, 1e-5);
  EXPECT_NEAR(odometry_.getY(), expected_y, 1e-5);
  EXPECT_NEAR(odometry_.getHeading(), 1.0, 1e-5);
}

TEST_F(OmniOdometryTest, TestSmallDtRejection)
{
  std::vector<double> wheel_vels = {1.0, 1.0, 1.0, 1.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1e-7);

  EXPECT_FALSE(result);
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
}

TEST_F(OmniOdometryTest, TestOpenLoopUpdate)
{
  // Directly feed vx=2.0, vy=0.5, w=1.0, dt=1.0 to bypass SVD math
  bool result = odometry_.try_update_open_loop(2.0, 0.5, 1.0, 1.0);

  EXPECT_TRUE(result);

  const double expected_x = (2.0 / 1.0) * std::sin(1.0) + (0.5 / 1.0) * (std::cos(1.0) - 1.0);
  const double expected_y = -(2.0 / 1.0) * (std::cos(1.0) - 1.0) + (0.5 / 1.0) * std::sin(1.0);

  EXPECT_NEAR(odometry_.getX(), expected_x, 1e-5);
  EXPECT_NEAR(odometry_.getY(), expected_y, 1e-5);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 1.0);

  EXPECT_DOUBLE_EQ(odometry_.getLinearXVel(), 2.0);
  EXPECT_DOUBLE_EQ(odometry_.getLinearYVel(), 0.5);
  EXPECT_DOUBLE_EQ(odometry_.getAngularVel(), 1.0);
}

TEST_F(OmniOdometryTest, TestUpdateFromPosition)
{
  // Feed position increments that equate to Vx=1.0 over dt=1.0
  std::vector<double> wheel_pos = {0.0, 10.0, 0.0, -10.0};

  bool result = odometry_.update_from_pos(wheel_pos, 1.0);

  EXPECT_TRUE(result);
  EXPECT_NEAR(odometry_.getX(), 1.0, 1e-5);
  EXPECT_NEAR(odometry_.getLinearXVel(), 1.0, 1e-5);
}

TEST_F(OmniOdometryTest, TestReset)
{
  // 1. Move the robot
  std::vector<double> wheel_vels = {0.0, 10.0, 0.0, -10.0};
  bool result = odometry_.update_from_vel(wheel_vels, 1.0);
  EXPECT_TRUE(result);
  EXPECT_NE(odometry_.getX(), 0.0);

  // 2. Reset
  odometry_.setOdometry(0.0, 0.0, 0.0);

  // 3. Verify position is cleared
  EXPECT_DOUBLE_EQ(odometry_.getX(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getY(), 0.0);
  EXPECT_DOUBLE_EQ(odometry_.getHeading(), 0.0);
}
