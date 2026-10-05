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
#include <stdexcept>
#include <string>

#include "steering_controllers_library/steering_kinematics.hpp"

namespace
{
constexpr double TOLERANCE = 1e-10;
constexpr double WHEEL_RADIUS = 0.31;
constexpr double WHEEL_BASE = 1.7;
constexpr double STEERING_TRACK = 1.1;
constexpr double TRACTION_TRACK = 1.5;

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

struct WheelState
{
  double center_traction;
  double right_traction;
  double left_traction;
  double center_steering;
  double right_steering;
  double left_steering;
};

WheelState wheel_state_from_twist(const double linear, const double angular)
{
  const double center_steering = std::atan(angular * WHEEL_BASE / linear);
  const double turning_radius = linear / angular;
  const double center_traction = linear / WHEEL_RADIUS;
  const double right_traction =
    center_traction * (turning_radius + TRACTION_TRACK * 0.5) / turning_radius;
  const double left_traction =
    center_traction * (turning_radius - TRACTION_TRACK * 0.5) / turning_radius;
  const double right_steering = std::atan2(
    2.0 * WHEEL_BASE * std::sin(center_steering),
    2.0 * WHEEL_BASE * std::cos(center_steering) + STEERING_TRACK * std::sin(center_steering));
  const double left_steering = std::atan2(
    2.0 * WHEEL_BASE * std::sin(center_steering),
    2.0 * WHEEL_BASE * std::cos(center_steering) - STEERING_TRACK * std::sin(center_steering));
  return {center_traction, right_traction, left_traction,
          center_steering, right_steering, left_steering};
}

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

auto PoseNear(const Pose & expected)
{
  return ::testing::AllOf(
    ::testing::Field("x", &Pose::x, ::testing::DoubleNear(expected.x, TOLERANCE)),
    ::testing::Field("y", &Pose::y, ::testing::DoubleNear(expected.y, TOLERANCE)),
    ::testing::Field(
      "heading", &Pose::heading, ::testing::DoubleNear(expected.heading, TOLERANCE)));
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

    EXPECT_THAT(
      (Pose{odom.get_x(), odom.get_y(), odom.get_heading()}),
      PoseNear(integrate_twist(initial, linear, angular, dt)));
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

class SteeringOdometryFeedbackTest : public ::testing::TestWithParam<SteeringConfiguration>
{
protected:
  void configure(steering_kinematics::SteeringKinematics & odom) const
  {
    odom.set_wheel_params(WHEEL_RADIUS, WHEEL_BASE, STEERING_TRACK, TRACTION_TRACK);
    odom.set_odometry_type(GetParam().type);
  }

  bool update_from_velocity(
    steering_kinematics::SteeringKinematics & odom, const double linear, const double angular,
    const double dt) const
  {
    const auto wheels = wheel_state_from_twist(linear, angular);
    switch (GetParam().type)
    {
      case steering_kinematics::BICYCLE_CONFIG:
        return odom.update_from_velocity(wheels.center_traction, wheels.center_steering, dt);
      case steering_kinematics::TRICYCLE_CONFIG:
        return odom.update_from_velocity(
          wheels.right_traction, wheels.left_traction, wheels.center_steering, dt);
      case steering_kinematics::ACKERMANN_CONFIG:
        return odom.update_from_velocity(
          wheels.right_traction, wheels.left_traction, wheels.right_steering, wheels.left_steering,
          dt);
      default:
        throw std::logic_error("unexpected test configuration");
    }
  }

  bool update_from_position(
    steering_kinematics::SteeringKinematics & odom, const double linear, const double angular,
    const double wheel_position_time, const double dt) const
  {
    const auto wheels = wheel_state_from_twist(linear, angular);
    switch (GetParam().type)
    {
      case steering_kinematics::BICYCLE_CONFIG:
        return odom.update_from_position(
          wheels.center_traction * wheel_position_time, wheels.center_steering, dt);
      case steering_kinematics::TRICYCLE_CONFIG:
        return odom.update_from_position(
          wheels.right_traction * wheel_position_time, wheels.left_traction * wheel_position_time,
          wheels.center_steering, dt);
      case steering_kinematics::ACKERMANN_CONFIG:
        return odom.update_from_position(
          wheels.right_traction * wheel_position_time, wheels.left_traction * wheel_position_time,
          wheels.right_steering, wheels.left_steering, dt);
      default:
        throw std::logic_error("unexpected test configuration");
    }
  }
};

// cppcheck-suppress syntaxError
// Cppcheck does not expand the GoogleTest parameterized-test macro.
TEST_P(SteeringOdometryFeedbackTest, velocity_feedback_matches_body_twist)
{
  steering_kinematics::SteeringKinematics odom(1);
  configure(odom);
  const Pose initial{-0.2, 0.5, 0.25};
  odom.set_odometry(initial.x, initial.y, initial.heading);

  constexpr double linear = 0.8;
  constexpr double angular = 0.32;
  constexpr double dt = 0.25;
  ASSERT_TRUE(update_from_velocity(odom, linear, angular, dt));

  EXPECT_THAT(
    (Pose{odom.get_x(), odom.get_y(), odom.get_heading()}),
    PoseNear(integrate_twist(initial, linear, angular, dt)));
  EXPECT_NEAR(odom.get_linear(), linear, TOLERANCE);
  EXPECT_NEAR(odom.get_angular(), angular, TOLERANCE);
}

TEST_P(SteeringOdometryFeedbackTest, position_feedback_uses_incremental_wheel_state)
{
  steering_kinematics::SteeringKinematics odom(1);
  configure(odom);
  const Pose initial{0.3, -0.6, -0.2};
  odom.set_odometry(initial.x, initial.y, initial.heading);

  constexpr double linear = 0.9;
  constexpr double angular = -0.28;
  constexpr double dt = 0.4;
  ASSERT_TRUE(update_from_position(odom, linear, angular, dt, dt));
  ASSERT_TRUE(update_from_position(odom, linear, angular, 2.0 * dt, dt));

  EXPECT_THAT(
    (Pose{odom.get_x(), odom.get_y(), odom.get_heading()}),
    PoseNear(integrate_twist(initial, linear, angular, 2.0 * dt)));
  EXPECT_NEAR(odom.get_linear(), linear, TOLERANCE);
  EXPECT_NEAR(odom.get_angular(), angular, TOLERANCE);
}

INSTANTIATE_TEST_SUITE_P(
  SteeringFeedbackConfigurations, SteeringOdometryFeedbackTest,
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
