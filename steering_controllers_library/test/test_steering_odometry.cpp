// Copyright (c) 2023, Virtual Vehicle Research GmbH
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

#include "steering_controllers_library/steering_odometry.hpp"

TEST(TestSteeringOdometry, initialize)
{
  EXPECT_NO_THROW(steering_odometry::SteeringOdometry());

  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 3.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  EXPECT_DOUBLE_EQ(odom.get_heading(), 0.);
  EXPECT_DOUBLE_EQ(odom.get_x(), 0.);
  EXPECT_DOUBLE_EQ(odom.get_y(), 0.);
}

TEST(TestSteeringOdometry, ackermann_fwd_kin_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(2., 0., 0.5);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 2.);
  EXPECT_DOUBLE_EQ(odom.get_x(), 1.);
  EXPECT_DOUBLE_EQ(odom.get_y(), 0.);
}

TEST(TestSteeringOdometry, ackermann_fwd_kin_angular_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., 1., 1.);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 1.);
  EXPECT_DOUBLE_EQ(odom.get_angular(), 1.);

  EXPECT_GT(odom.get_x(), 0);  // pos x
  EXPECT_GT(odom.get_y(), 0);  // pos y, ie. left
}

TEST(TestSteeringOdometry, ackermann_fwd_kin_angular_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., -1., 1.);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 1.);
  EXPECT_DOUBLE_EQ(odom.get_angular(), -1.);
  EXPECT_GT(odom.get_x(), 0);  // pos x
  EXPECT_LT(odom.get_y(), 0);  // neg y ie. right
}

TEST(TestSteeringOdometry, ackermann_back_kin_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0.);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_EQ(cmd0[0], cmd0[1]);   // linear
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_EQ(cmd1[0], cmd1[1]);   // no steering
  EXPECT_EQ(cmd1[0], 0);
}

TEST(TestSteeringOdometry, ackermann_back_kin_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_GT(cmd0[0], cmd0[1]);   // right (outer) > left (inner)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], cmd1[1]);   // right (outer) < left (inner)
  EXPECT_GT(cmd1[0], 0);
}

TEST(TestSteeringOdometry, ackermann_back_kin_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left outer)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);                     // steer
  EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
  EXPECT_LT(cmd1[0], 0);
}

TEST(TestSteeringOdometry, swerve_back_kin_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::SWERVE_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0.);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_EQ(cmd0[0], cmd0[1]);   // linear
  EXPECT_EQ(cmd0[0], cmd0[2]);   // linear
  EXPECT_EQ(cmd0[0], cmd0[3]);   // linear
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_EQ(cmd1[0], cmd1[1]);   // no steering
  EXPECT_EQ(cmd1[0], cmd1[2]);   // no steering
  EXPECT_EQ(cmd1[0], cmd1[3]);   // no steering
  EXPECT_EQ(cmd1[0], 0);
}

TEST(TestSteeringOdometry, swerve_back_kin_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::SWERVE_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_GT(cmd0[0], cmd0[1]);   // front right (outer) > front left (inner)
  EXPECT_GT(cmd0[0], 0);
  EXPECT_EQ(cmd0[2], cmd0[0]);   // rear right == front right
  EXPECT_EQ(cmd0[3], cmd0[1]);   // rear left  == front left
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], cmd1[1]);   // front right (outer) < front left (inner)
  EXPECT_GT(cmd1[0], 0);
  EXPECT_EQ(cmd1[2], -cmd1[0]);   // rear right == - front right
  EXPECT_EQ(cmd1[3], -cmd1[1]);   // rear left  == - front left
}

TEST(TestSteeringOdometry, swerve_back_kin_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::SWERVE_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_LT(cmd0[0], cmd0[1]);   // front right (inner) < front left (outer)
  EXPECT_GT(cmd0[0], 0);
  EXPECT_EQ(cmd0[2], cmd0[0]);   // rear right == front right
  EXPECT_EQ(cmd0[3], cmd0[1]);   // rear left  == front left
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], cmd1[1]);   // front right (outer) < front left (inner)
  EXPECT_LT(cmd1[0], 0);
  EXPECT_EQ(cmd1[2], -cmd1[0]);   // rear right == - front right
  EXPECT_EQ(cmd1[3], -cmd1[1]);   // rear left  == - front left
}

TEST(TestSteeringOdometry, bicycle_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::BICYCLE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

TEST(TestSteeringOdometry, tricycle_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

TEST(TestSteeringOdometry, ackermann_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

TEST(TestSteeringOdometry, swerve_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::SWERVE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}


