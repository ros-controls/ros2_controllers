// Copyright 2025 (your name or organization)
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

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>


namespace swerve_drive_controller
{

/**
 * @brief Struct to represent the kinematics command for a single wheel .
 */

struct WheelCommand
{
  double steering_angle;  // Steering angle in radians
  double drive_velocity;  // Drive velocity in meters per second
};

/**
 * @brief Struct to represent the odometry state of the robot.
 */

struct OdometryState
{
  double x;      // X position in meters
  double y;      // Y position in meters
  double theta;  // Orientation (yaw) in radians
};

class SwerveDriveKinematics
{
public:
  /**
   * @brief Constructor for the kinematics solver.
   * @param wheel_positions Array of (x, y) positions of the wheels relative to the robot's center.
   */

  explicit SwerveDriveKinematics(const std::array<std::pair<double, double>, 4> & wheel_positions);
  /**
   * @brief Compute the wheel commands based on robot velocity commands.
   * @param linear_velocity_x Linear velocity in the x direction (m/s).
   * @param linear_velocity_y Linear velocity in the y direction (m/s).
   * @param angular_velocity_z Angular velocity about the z-axis (rad/s).
   * @return Array of wheel commands (steering angles and drive velocities).
   */

  std::array<WheelCommand, 4> compute_wheel_commands(
    double linear_velocity_x, double linear_velocity_y, double angular_velocity_z);

  /**
   * @brief Update the odometry based on wheel velocities and elapsed time.
   * @param wheel_velocities Array of measured wheel velocities (m/s).
   * @param steering_angles Array of measured steering angles (radians).
   * @param dt Time step (seconds).
   * @return Updated odometry state.
   */

  OdometryState update_odometry(
    const std::array<double, 4> & wheel_velocities_, const std::array<double, 4> & steering_angles_,
    double dt);

private:
  std::array<std::pair<double, double>, 4> wheel_positions_;  // Wheel Positions
  OdometryState odometry_;                                    // Current Odometry of the robot

  /**
   * @brief Normalize an angle to the range [-pi, pi].
   * @param angle input in radians.
   * @return Normalized angle in radians.
   */

  double normalize_angle(double angle);
};
}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
