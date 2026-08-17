// Copyright (c) 2023, Virtual Vehicle Research GmbH
// Copyright (c) 2026, Dylan Gallagher
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
#include <cmath>
#include <string>

#include "steering_controllers_library/steering_kinematics.hpp"

namespace
{
constexpr double TOLERANCE = 1e-10;

struct SteeringConfiguration
{
  unsigned int type;
  const char * name;
};

struct Pose
{
  double x;
  double y;
  double heading;
};

Pose integrate_twist(
  const Pose & initial, const double linear, const double angular, const double dt)
{
  if (steering_kinematics::is_close_to_zero(angular * dt))
  {
    const double middle_heading = initial.heading + angular * dt * 0.5;
    return {
      initial.x + linear * std::cos(middle_heading) * dt,
      initial.y + linear * std::sin(middle_heading) * dt, initial.heading + angular * dt};
  }

  const double final_heading = initial.heading + angular * dt;
  const double turning_radius = linear / angular;
  return {
    initial.x + turning_radius * (std::sin(final_heading) - std::sin(initial.heading)),
    initial.y - turning_radius * (std::cos(final_heading) - std::cos(initial.heading)),
    final_heading};
}

void expect_pose(const steering_kinematics::SteeringKinematics & odom, const Pose & expected)
{
  EXPECT_NEAR(odom.get_x(), expected.x, TOLERANCE);
  EXPECT_NEAR(odom.get_y(), expected.y, TOLERANCE);
  EXPECT_NEAR(odom.get_heading(), expected.heading, TOLERANCE);
}

class SteeringOdometryIntegratorTest : public ::testing::TestWithParam<SteeringConfiguration>
{
protected:
  void expect_open_loop(
    const Pose & initial, const double linear, const double angular, const double dt) const
  {
    SCOPED_TRACE(
      ::testing::Message() << "initial=" << initial.x << "," << initial.y << "," << initial.heading
                           << " twist=" << linear << "," << angular << " dt=" << dt);
    steering_kinematics::SteeringKinematics odom(1);
    odom.set_odometry_type(GetParam().type);
    odom.set_odometry(initial.x, initial.y, initial.heading);
    odom.update_open_loop(linear, angular, dt);

    expect_pose(odom, integrate_twist(initial, linear, angular, dt));
    EXPECT_DOUBLE_EQ(odom.get_linear(), linear);
    EXPECT_DOUBLE_EQ(odom.get_angular(), angular);
  }
};

std::string configuration_name(const ::testing::TestParamInfo<SteeringConfiguration> & information)
{
  return information.param.name;
}

// cppcheck-suppress syntaxError
// Cppcheck does not expand the GoogleTest parameterized-test macro.
TEST_P(SteeringOdometryIntegratorTest, integrates_open_loop_straight)
{
  // Retain the former Ackermann-only zero-pose case and extend it to every configuration.
  expect_open_loop(Pose{0.0, 0.0, 0.0}, 2.0, 0.0, 0.5);
  expect_open_loop(Pose{-1.2, 0.7, 0.4}, 1.1, 0.0, 0.35);
}

TEST_P(SteeringOdometryIntegratorTest, integrates_open_loop_arcs)
{
  for (const double direction : {-1.0, 1.0})
  {
    // Retain both former Ackermann-only turn directions with a stronger exact-pose oracle.
    expect_open_loop(Pose{0.0, 0.0, 0.0}, 1.0, direction, 1.0);
    expect_open_loop(Pose{0.8, -0.45, -0.3}, 1.3, direction * 0.45, 0.6);
  }
}

INSTANTIATE_TEST_SUITE_P(
  SteeringConfigurations, SteeringOdometryIntegratorTest,
  ::testing::Values(
    SteeringConfiguration{steering_kinematics::BICYCLE_CONFIG, "Bicycle"},
    SteeringConfiguration{steering_kinematics::TRICYCLE_CONFIG, "Tricycle"},
    SteeringConfiguration{steering_kinematics::ACKERMANN_CONFIG, "Ackermann"}),
  configuration_name);
}  // namespace

TEST(TestSteeringOdometry, initialize)
{
  EXPECT_NO_THROW(steering_kinematics::SteeringKinematics());

  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 3.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);
  EXPECT_DOUBLE_EQ(odom.get_heading(), 0.);
  EXPECT_DOUBLE_EQ(odom.get_x(), 0.);
  EXPECT_DOUBLE_EQ(odom.get_y(), 0.);
}

// ----------------- Ackermann -----------------

TEST(TestSteeringOdometry, ackermann_odometry)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

TEST(TestSteeringOdometry, ackermann_IK_linear)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::ACKERMANN_CONFIG);

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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::BICYCLE_CONFIG);
  odom.update_open_loop(1., 0., 1.);
  auto cmd = odom.get_commands(1., 0., true);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_DOUBLE_EQ(cmd1[0], 0);    // no steering
}

TEST(TestSteeringOdometry, bicycle_IK_left)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::BICYCLE_CONFIG);
  odom.update_from_position(0., 0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., 0.1, false);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_GT(cmd1[0], 0);           // right steering
}

TEST(TestSteeringOdometry, bicycle_IK_right)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::BICYCLE_CONFIG);
  odom.update_from_position(0., -0.2, 1.);  // assume already turn
  auto cmd = odom.get_commands(1., -0.1, false);
  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_LT(cmd1[0], 0);           // left steering
}

TEST(TestSteeringOdometry, bicycle_IK_right_steering_limited)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::BICYCLE_CONFIG);

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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_kinematics::BICYCLE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}

// ----------------- tricycle -----------------

TEST(TestSteeringOdometry, tricycle_IK_linear)
{
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::TRICYCLE_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::TRICYCLE_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::TRICYCLE_CONFIG);
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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 2., 1.);
  odom.set_odometry_type(steering_kinematics::TRICYCLE_CONFIG);

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
  steering_kinematics::SteeringKinematics odom(1);
  odom.set_wheel_params(1., 1., 1.);
  odom.set_odometry_type(steering_kinematics::TRICYCLE_CONFIG);
  ASSERT_TRUE(odom.update_from_velocity(1., 1., .1, .1));
  EXPECT_NEAR(odom.get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom.get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom.get_x(), .1, 1e-3);
  EXPECT_NEAR(odom.get_heading(), .01, 1e-3);
}
