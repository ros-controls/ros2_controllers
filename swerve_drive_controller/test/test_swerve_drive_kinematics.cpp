// Copyright 2025 ros2_control development team
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

#include <array>
#include <cmath>

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

using swerve_drive_controller::OdometryState;
using swerve_drive_controller::SwerveDriveKinematics;

namespace
{
constexpr double WHEELBASE = 0.5;
constexpr double TRACKWIDTH = 0.4;
constexpr double WHEEL_RADIUS = 0.05;
constexpr double TOLERANCE = 1e-9;
}  // namespace

class SwerveDriveKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override { kinematics_.calculate_wheel_position(WHEELBASE, TRACKWIDTH); }

  // Feed the wheel commands for one robot velocity into the odometry, steps times
  OdometryState drive(double vx, double vy, double wz, double dt, int steps)
  {
    const auto commands = kinematics_.compute_wheel_commands(vx, vy, wz, WHEEL_RADIUS);
    std::array<double, 4> wheel_speeds{};
    std::array<double, 4> steering_angles{};
    for (std::size_t i = 0; i < 4; i++)
    {
      wheel_speeds[i] = commands[i].drive_velocity;
      steering_angles[i] = commands[i].steering_angle;
    }

    OdometryState state{};
    for (int step = 0; step < steps; step++)
    {
      state = kinematics_.update_odometry(wheel_speeds, steering_angles, dt);
    }
    return state;
  }

  SwerveDriveKinematics kinematics_;
};

TEST_F(SwerveDriveKinematicsTest, forward_command_points_all_wheels_straight)
{
  const auto commands = kinematics_.compute_wheel_commands(1.0, 0.0, 0.0, WHEEL_RADIUS);

  for (const auto & command : commands)
  {
    EXPECT_NEAR(command.steering_angle, 0.0, TOLERANCE);
    EXPECT_NEAR(command.drive_velocity, 1.0, TOLERANCE);
    EXPECT_NEAR(command.drive_angular_velocity, 1.0 / WHEEL_RADIUS, TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, sideways_command_points_all_wheels_left)
{
  const auto commands = kinematics_.compute_wheel_commands(0.0, 0.5, 0.0, WHEEL_RADIUS);

  for (const auto & command : commands)
  {
    EXPECT_NEAR(command.steering_angle, M_PI_2, TOLERANCE);
    EXPECT_NEAR(command.drive_velocity, 0.5, TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, diagonal_command_points_all_wheels_at_45_degrees)
{
  const auto commands = kinematics_.compute_wheel_commands(0.3, 0.3, 0.0, WHEEL_RADIUS);

  for (const auto & command : commands)
  {
    EXPECT_NEAR(command.steering_angle, M_PI_4, TOLERANCE);
    EXPECT_NEAR(command.drive_velocity, std::hypot(0.3, 0.3), TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, turn_in_place_points_wheels_around_the_center)
{
  const double wz = 1.0;
  const auto commands = kinematics_.compute_wheel_commands(0.0, 0.0, wz, WHEEL_RADIUS);

  // Order: front_left, front_right, rear_left, rear_right
  const std::array<double, 4> wheel_x = {
    {WHEELBASE / 2, WHEELBASE / 2, -WHEELBASE / 2, -WHEELBASE / 2}};
  const std::array<double, 4> wheel_y = {
    {TRACKWIDTH / 2, -TRACKWIDTH / 2, TRACKWIDTH / 2, -TRACKWIDTH / 2}};
  const double distance_to_center = std::hypot(WHEELBASE / 2, TRACKWIDTH / 2);

  for (std::size_t i = 0; i < 4; i++)
  {
    // Every wheel moves at the same speed on a circle around the robot center
    EXPECT_NEAR(commands[i].drive_velocity, wz * distance_to_center, TOLERANCE);

    // so it points at a right angle to the line from the center to the wheel
    const double wheel_direction_x = std::cos(commands[i].steering_angle);
    const double wheel_direction_y = std::sin(commands[i].steering_angle);
    const double dot_product = wheel_direction_x * wheel_x[i] + wheel_direction_y * wheel_y[i];
    EXPECT_NEAR(dot_product, 0.0, TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, optimize_flips_wheel_instead_of_turning_more_than_90_degrees)
{
  // Driving backward: the target angle is 180 degrees away from the current 0 degrees
  const auto commands = kinematics_.compute_wheel_commands(-1.0, 0.0, 0.0, WHEEL_RADIUS);
  const std::array<double, 4> current_steering_angles = {{0.0, 0.0, 0.0, 0.0}};

  const auto optimized = kinematics_.optimize_wheel_commands(commands, current_steering_angles);

  for (const auto & command : optimized)
  {
    EXPECT_NEAR(std::abs(command.steering_angle), 0.0, TOLERANCE);
    EXPECT_NEAR(command.drive_velocity, -1.0, TOLERANCE);
    EXPECT_NEAR(command.drive_angular_velocity, -1.0 / WHEEL_RADIUS, TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, optimize_keeps_small_steering_changes)
{
  const auto commands = kinematics_.compute_wheel_commands(0.3, 0.3, 0.0, WHEEL_RADIUS);
  const std::array<double, 4> current_steering_angles = {{0.0, 0.0, 0.0, 0.0}};

  const auto optimized = kinematics_.optimize_wheel_commands(commands, current_steering_angles);

  for (std::size_t i = 0; i < 4; i++)
  {
    EXPECT_NEAR(optimized[i].steering_angle, commands[i].steering_angle, TOLERANCE);
    EXPECT_NEAR(optimized[i].drive_velocity, commands[i].drive_velocity, TOLERANCE);
  }
}

TEST_F(SwerveDriveKinematicsTest, odometry_straight_line)
{
  // 1 m/s forward for 10 x 0.1 s
  const auto state = drive(1.0, 0.0, 0.0, 0.1, 10);

  EXPECT_NEAR(state.x, 1.0, TOLERANCE);
  EXPECT_NEAR(state.y, 0.0, TOLERANCE);
  EXPECT_NEAR(state.theta, 0.0, TOLERANCE);
  EXPECT_NEAR(state.vx, 1.0, TOLERANCE);
  EXPECT_NEAR(state.vy, 0.0, TOLERANCE);
  EXPECT_NEAR(state.wz, 0.0, TOLERANCE);
}

TEST_F(SwerveDriveKinematicsTest, odometry_sideways_line)
{
  // 0.5 m/s to the left for 20 x 0.1 s
  const auto state = drive(0.0, 0.5, 0.0, 0.1, 20);

  EXPECT_NEAR(state.x, 0.0, TOLERANCE);
  EXPECT_NEAR(state.y, 1.0, TOLERANCE);
  EXPECT_NEAR(state.theta, 0.0, TOLERANCE);
}

TEST_F(SwerveDriveKinematicsTest, odometry_turn_in_place)
{
  // 1 rad/s for 10 x 0.1 s
  const auto state = drive(0.0, 0.0, 1.0, 0.1, 10);

  EXPECT_NEAR(state.x, 0.0, TOLERANCE);
  EXPECT_NEAR(state.y, 0.0, TOLERANCE);
  EXPECT_NEAR(state.theta, 1.0, TOLERANCE);
  EXPECT_NEAR(state.wz, 1.0, TOLERANCE);
}

TEST_F(SwerveDriveKinematicsTest, odometry_half_circle)
{
  // 1 m/s forward while turning left at 1 rad/s drives a circle with 1 m radius around (0, 1).
  // After pi seconds the robot is on the other side of the circle, facing backward.
  const int steps = 1000;
  const auto state = drive(1.0, 0.0, 1.0, M_PI / steps, steps);

  EXPECT_NEAR(state.x, 0.0, 1e-6);
  EXPECT_NEAR(state.y, 2.0, 1e-6);
  EXPECT_NEAR(state.theta, M_PI, 1e-6);
}

TEST_F(SwerveDriveKinematicsTest, odometry_heading_is_not_wrapped)
{
  // A full turn and a bit more: heading keeps counting up like in the other drive controllers
  const auto state = drive(0.0, 0.0, 1.0, 0.1, 70);

  EXPECT_NEAR(state.theta, 7.0, TOLERANCE);
}

TEST_F(SwerveDriveKinematicsTest, odometry_ignores_zero_time_step)
{
  drive(1.0, 0.0, 0.0, 0.1, 10);
  const auto state = drive(1.0, 0.0, 0.0, 0.0, 5);

  EXPECT_NEAR(state.x, 1.0, TOLERANCE);
  EXPECT_NEAR(state.y, 0.0, TOLERANCE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
