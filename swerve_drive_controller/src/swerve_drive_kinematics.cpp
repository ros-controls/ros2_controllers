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

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller
{

SwerveDriveKinematics::SwerveDriveKinematics() : odometry_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} {}

void SwerveDriveKinematics::calculate_wheel_position(
  double wheel_base, double track_width, double x_offset, double y_offset)
{
  double half_length = wheel_base / 2.0;
  double half_width = track_width / 2.0;

  wheel_positions_[0] = {half_length - x_offset, half_width - y_offset};    // Front Left  (+x, +y)
  wheel_positions_[1] = {half_length - x_offset, -half_width - y_offset};   // Front Right (+x, -y)
  wheel_positions_[2] = {-half_length - x_offset, half_width - y_offset};   // Rear Left   (-x, +y)
  wheel_positions_[3] = {-half_length - x_offset, -half_width - y_offset};  // Rear Right  (-x, -y)
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z,
  double wheel_radius)
{
  std::array<WheelCommand, 4> wheel_commands;

  if (wheel_radius <= 0.0)
  {
    std::cerr << "invalid wheel_radius <= 0.0\n";
    // fallthrough: compute but set angular velocities to 0 to avoid div-by-zero
  }

  for (std::size_t i = 0; i < 4; i++)
  {
    const auto & [wx, wy] = wheel_positions_[i];

    double vx = linear_velocity_x - angular_velocity_z * wy;
    double vy = linear_velocity_y + angular_velocity_z * wx;

    double linear_speed = std::hypot(vx, vy);
    double steering = std::atan2(vy, vx);

    wheel_commands[i].drive_velocity = linear_speed;

    if (wheel_radius > 0.0)
    {
      wheel_commands[i].drive_angular_velocity = linear_speed / wheel_radius;  // rad/s
    }
    else
    {
      wheel_commands[i].drive_angular_velocity = 0.0;  // safe fallback
    }

    wheel_commands[i].steering_angle = steering;
  }

  return wheel_commands;
}

std::array<WheelCommand, 4> SwerveDriveKinematics::optimize_wheel_commands(
  const std::array<WheelCommand, 4> & wheel_commands,
  const std::array<double, 4> & current_steering_angles)
{
  std::array<WheelCommand, 4> optimized_commands = wheel_commands;

  for (std::size_t i = 0; i < 4; i++)
  {
    double target_angle = wheel_commands[i].steering_angle;
    double current_angle = current_steering_angles[i];

    double angle_diff = angles::shortest_angular_distance(current_angle, target_angle);

    if (std::abs(angle_diff) > M_PI_2)
    {
      optimized_commands[i].drive_velocity = -wheel_commands[i].drive_velocity;
      optimized_commands[i].drive_angular_velocity = -wheel_commands[i].drive_angular_velocity;

      optimized_commands[i].steering_angle = angles::normalize_angle(target_angle + M_PI);
    }
  }

  return optimized_commands;
}

OdometryState SwerveDriveKinematics::update_odometry(
  const std::array<double, 4> & wheel_velocities, const std::array<double, 4> & steering_angles,
  double dt)
{
  // Compute robot-centric velocity (assuming perfect wheel control)
  double vx_sum = 0.0, vy_sum = 0.0, wz_sum = 0.0;
  for (std::size_t i = 0; i < 4; i++)
  {
    double vx = wheel_velocities[i] * std::cos(steering_angles[i]);
    double vy = wheel_velocities[i] * std::sin(steering_angles[i]);

    // Accumulate contributions to linear and angular velocities
    vx_sum += vx;
    vy_sum += vy;

    wz_sum += (vy * wheel_positions_[i].first - vx * wheel_positions_[i].second);
  }

  double vx_robot = vx_sum / 4.0;
  double vy_robot = vy_sum / 4.0;

  double wz_denominator = 0.0;
  for (std::size_t i = 0; i < 4; i++)
  {
    wz_denominator +=
      (wheel_positions_[i].first * wheel_positions_[i].first +
       wheel_positions_[i].second * wheel_positions_[i].second);
  }
  double wz_robot = wz_sum / wz_denominator;

  // Compute linear displacement in robot frame
  double dx_robot = vx_robot * dt;
  double dy_robot = vy_robot * dt;
  double dtheta = wz_robot * dt;

  // Integrate using Exact integration for rotation, RK2 fallback for near-zero angular velocity
  if (std::fabs(dtheta) < 1e-6)
  {
    // RK2 (Runge-Kutta 2nd order) integration for small angular changes
    // Use midpoint heading for better accuracy
    double mid_theta = odometry_.theta + dtheta * 0.5;
    double cos_mid = std::cos(mid_theta);
    double sin_mid = std::sin(mid_theta);

    odometry_.x += dx_robot * cos_mid - dy_robot * sin_mid;
    odometry_.y += dx_robot * sin_mid + dy_robot * cos_mid;
    odometry_.theta += dtheta;
  }
  else
  {
    // Exact integration for arc motion
    // When the robot rotates, it follows an arc trajectory
    // This computes the exact displacement along that arc
    double theta_old = odometry_.theta;
    double theta_new = theta_old + dtheta;

    double sin_new = std::sin(theta_new);
    double sin_old = std::sin(theta_old);
    double cos_new = std::cos(theta_new);
    double cos_old = std::cos(theta_old);

    // Exact integration formula for simultaneous translation and rotation
    // Derived from integrating velocity over the arc
    odometry_.x += (dx_robot * (sin_new - sin_old) + dy_robot * (cos_new - cos_old)) / dtheta;
    odometry_.y += (dx_robot * (cos_old - cos_new) + dy_robot * (sin_new - sin_old)) / dtheta;
    odometry_.theta = theta_new;
  }

  // Normalize theta to [-pi, pi]
  odometry_.theta = angles::normalize_angle(odometry_.theta);
  odometry_.vx = vx_robot;
  odometry_.vy = vy_robot;
  odometry_.wz = wz_robot;
  return odometry_;
}
}  // namespace swerve_drive_controller
