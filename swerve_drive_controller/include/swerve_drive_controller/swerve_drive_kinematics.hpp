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

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_

#include <angles/angles.h>
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
  double steering_angle;          // Steering angle in radians
  double drive_velocity;          // Drive velocity in meters per second (m/s)
  double drive_angular_velocity;  // Drive angular velocity in radians per second (rad/s)
};

/**
 * @brief Struct to represent the odometry state of the robot.
 */

struct OdometryState
{
  double x;      // X position in meters
  double y;      // Y position in meters
  double theta;  // Orientation (yaw) in radians
  double vx;     // Linear velocity in the x direction (m/s)
  double vy;     // Linear velocity in the y direction (m/s)
  double wz;     // Angular velocity about the z-axis (rad/s)
};

class SwerveDriveKinematics
{
public:
  /// @brief Default Constructor
  SwerveDriveKinematics();

  /**
   * @brief Sets necessary params required for kinematics calculation.
   * @param wheel_base Distance between front and rear axles (meters).
   * @param track_width Distance between left and right wheels (meters).
   * @param x_offset Optional global x offset of wheel positions.
   * @param y_offset Optional global y offset of wheel positions.
   * @attention order enforced as: front_left, front_right, rear_left, rear_right
   */
  void calculate_wheel_position(
    double wheel_base, double track_width, double x_offset = 0.0, double y_offset = 0.0);

  /**
   * @brief Compute the wheel commands based on robot velocity commands.
   * @param linear_velocity_x Linear velocity in the x direction (m/s).
   * @param linear_velocity_y Linear velocity in the y direction (m/s).
   * @param angular_velocity_z Angular velocity about the z-axis (rad/s).
   * @param wheel_radius Radius of the wheel (meters). Same radius used for all wheels.
   * @return Array of wheel commands (steering angles, drive linear velocities (m/s),
   *         and drive angular velocities (rad/s)).
   */
  std::array<WheelCommand, 4> compute_wheel_commands(
    double linear_velocity_x, double linear_velocity_y, double angular_velocity_z,
    double wheel_radius);

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
};
}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
