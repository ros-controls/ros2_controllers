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

#define _USE_MATH_DEFINES

#include <gmock/gmock.h>

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

// ----------------- Ackermann -----------------

TEST(TestSteeringOdometry, ackermann_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

TEST(TestSteeringOdometry, ackermann_odometry_openloop_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(2., 0., 0.5);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 2.);
  EXPECT_DOUBLE_EQ(odom.get_x(), 1.);
  EXPECT_DOUBLE_EQ(odom.get_y(), 0.);
}

TEST(TestSteeringOdometry, ackermann_odometry_openloop_angular_left)
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

TEST(TestSteeringOdometry, ackermann_odometry_openloop_angular_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., -1., 1.);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 1.);
  EXPECT_DOUBLE_EQ(odom.get_angular(), -1.);

  EXPECT_GT(odom.get_x(), 0);  // pos x
  EXPECT_LT(odom.get_y(), 0);  // neg y, ie. right
}

TEST(TestSteeringOdometry, ackermann_odometry_openloop_ackermanndrive_angular_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., 1., 1., false);
  EXPECT_DOUBLE_EQ(odom.get_linear(), 1.);
  double expected_angular = (1.0 / 2.0) * std::tan(1.0);
  EXPECT_NEAR(odom.get_angular(), expected_angular, 1e-6);
  EXPECT_GT(odom.get_x(), 0);  // pos x
  EXPECT_GT(odom.get_y(), 0);  // pos y, ie. left
}

TEST(TestSteeringOdometry, ackermann_IK_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0., true);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_EQ(cmd0[0], cmd0[1]);   // linear
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_EQ(cmd1[0], cmd1[1]);   // no steering
  EXPECT_EQ(cmd1[0], 0);
}

TEST(TestSteeringOdometry, ackermann_IK_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1, false);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_GT(cmd0[0], cmd0[1]);   // right (outer) > left (inner)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], cmd1[1]);   // right (outer) < left (inner)
  EXPECT_GT(cmd1[0], 0);
}

TEST(TestSteeringOdometry, ackermann_IK_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1, false);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left (outer)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);                     // steer
  EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
  EXPECT_LT(cmd1[0], 0);
}

TEST(TestSteeringOdometry, ackermann_IK_right_steering_limited)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

  {
    odom.update_from_position(0., -0.785, 1.);  // already steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto vel_cmd_steered = std::get<0>(cmd);            // vel
    EXPECT_LT(vel_cmd_steered[0], vel_cmd_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_steered[0], 0);
    auto cmd1 = std::get<1>(cmd);                     // steer
    EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
    EXPECT_LT(cmd1[0], 0);
  }

  std::vector<double> vel_cmd_not_steered;
  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, false);
    vel_cmd_not_steered = std::get<0>(cmd);                     // vel
    EXPECT_LT(vel_cmd_not_steered[0], vel_cmd_not_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_not_steered[0], 0);
    auto cmd1 = std::get<1>(cmd);                     // steer
    EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
    EXPECT_LT(cmd1[0], 0);
  }

  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto cmd0 = std::get<0>(cmd);  // vel
    EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left (outer)
    EXPECT_GT(cmd0[0], 0);
    // vel should be less than vel_cmd_not_steered now
    for (size_t i = 0; i < cmd0.size(); ++i)
    {
      EXPECT_LT(cmd0[i], vel_cmd_not_steered[i]);
    }
    auto cmd1 = std::get<1>(cmd);                     // steer
    EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
    EXPECT_LT(cmd1[0], 0);
  }
}

// ----------------- bicycle -----------------

TEST(TestSteeringOdometry, bicycle_IK_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::BICYCLE_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0., true);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_DOUBLE_EQ(cmd1[0], 0);    // no steering
}

TEST(TestSteeringOdometry, bicycle_IK_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::BICYCLE_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1, false);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_GT(cmd1[0], 0);           // right steering
}

TEST(TestSteeringOdometry, bicycle_IK_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::BICYCLE_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1, false);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_LT(cmd1[0], 0);           // left steering
}

TEST(TestSteeringOdometry, bicycle_IK_right_steering_limited)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::BICYCLE_CONFIG);

  {
    odom.update_from_position(0., -0.785, 1.);  // already steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto vel_cmd_steered = std::get<0>(cmd);    // vel
    EXPECT_DOUBLE_EQ(vel_cmd_steered[0], 1.0);  // equals linear
    auto cmd1 = std::get<1>(cmd);               // steer
    EXPECT_LT(cmd1[0], 0);
  }

  std::vector<double> vel_cmd_not_steered;
  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, false);
    vel_cmd_not_steered = std::get<0>(cmd);         // vel
    EXPECT_DOUBLE_EQ(vel_cmd_not_steered[0], 1.0);  // equals linear
    auto cmd1 = std::get<1>(cmd);                   // steer
    EXPECT_LT(cmd1[0], 0);
  }

  std::vector<double> vel_cmd_not_steered_limited;
  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    vel_cmd_not_steered_limited = std::get<0>(cmd);  // vel
    EXPECT_GT(vel_cmd_not_steered_limited[0], 0);
    // vel should be less than vel_cmd_not_steered now
    for (size_t i = 0; i < vel_cmd_not_steered_limited.size(); ++i)
    {
      EXPECT_LT(vel_cmd_not_steered_limited[i], vel_cmd_not_steered[i]);
    }
    auto cmd1 = std::get<1>(cmd);  // steer
    EXPECT_LT(cmd1[0], 0);
  }

  {
    // larger error -> check min of scale
    odom.update_from_position(0., M_PI, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto cmd0 = std::get<0>(cmd);  // vel
    EXPECT_GT(cmd0[0], 0);
    // vel should be less than vel_cmd_not_steered_limited now
    for (size_t i = 0; i < cmd0.size(); ++i)
    {
      EXPECT_LT(cmd0[i], vel_cmd_not_steered_limited[i]);
    }
    auto cmd1 = std::get<1>(cmd);  // steer
    EXPECT_LT(cmd1[0], 0);
  }
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

// ----------------- tricycle -----------------

TEST(TestSteeringOdometry, tricycle_IK_linear)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0., true);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_EQ(cmd0[0], cmd0[1]);   // linear
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_EQ(cmd1[0], 0);         // no steering
}

TEST(TestSteeringOdometry, tricycle_IK_left)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1, false);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_GT(cmd0[0], cmd0[1]);   // right (outer) > left (inner)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_GT(cmd1[0], 0);         // left steering
}

TEST(TestSteeringOdometry, tricycle_IK_right)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1, false);
  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left (outer)
  EXPECT_GT(cmd0[0], 0);
  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], 0);         // right steering
}

TEST(TestSteeringOdometry, tricycle_IK_right_steering_limited)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);

  {
    odom.update_from_position(0., -0.785, 1.);  // already steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto vel_cmd_steered = std::get<0>(cmd);            // vel
    EXPECT_LT(vel_cmd_steered[0], vel_cmd_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_steered[0], 0);
    auto cmd1 = std::get<1>(cmd);  // steer
    EXPECT_LT(cmd1[0], 0);
  }

  std::vector<double> vel_cmd_not_steered;
  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, false);
    vel_cmd_not_steered = std::get<0>(cmd);                     // vel
    EXPECT_LT(vel_cmd_not_steered[0], vel_cmd_not_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_not_steered[0], 0);
    auto cmd1 = std::get<1>(cmd);  // steer
    EXPECT_LT(cmd1[0], 0);
  }

  {
    odom.update_from_position(0., -0.1, 1.);  // not fully steered
    auto cmd = odom.get_commands(1., -0.5, false, true);
    auto cmd0 = std::get<0>(cmd);  // vel
    EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left (outer)
    EXPECT_GT(cmd0[0], 0);
    // vel should be less than vel_cmd_not_steered now
    for (size_t i = 0; i < cmd0.size(); ++i)
    {
      EXPECT_LT(cmd0[i], vel_cmd_not_steered[i]);
    }
    auto cmd1 = std::get<1>(cmd);  // steer
    EXPECT_LT(cmd1[0], 0);
  }
}

TEST(TestSteeringOdometry, tricycle_odometry)
{
  steering_odometry::SteeringOdometry odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_odometry::TRICYCLE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}
