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

  sum_of_squared_wheel_distances_ = 0.0;
  for (const auto & [wx, wy] : wheel_positions_)
  {
    sum_of_squared_wheel_distances_ += wx * wx + wy * wy;
  }
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z,
  double wheel_radius)
{
  std::array<WheelCommand, 4> wheel_commands;

  for (std::size_t i = 0; i < 4; i++)
  {
    const auto & [wx, wy] = wheel_positions_[i];

    double vx = linear_velocity_x - angular_velocity_z * wy;
    double vy = linear_velocity_y + angular_velocity_z * wx;

    double linear_speed = std::hypot(vx, vy);
    double steering = std::atan2(vy, vx);

    wheel_commands[i].drive_velocity = linear_speed;

    wheel_commands[i].drive_angular_velocity = linear_speed / wheel_radius;  // rad/s

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

  odometry_.vx = vx_sum / 4.0;
  odometry_.vy = vy_sum / 4.0;
  odometry_.wz = wz_sum / sum_of_squared_wheel_distances_;

  if (std::fabs(dt) < 1e-6)
  {
    return odometry_;
  }

  const double dx = odometry_.vx * dt;
  const double dy = odometry_.vy * dt;
  const double dtheta = odometry_.wz * dt;

  if (std::fabs(dtheta) < 1e-6)
  {
    // For very small heading changes, approximate to straight-line motion
    odometry_.x += dx * std::cos(odometry_.theta) - dy * std::sin(odometry_.theta);
    odometry_.y += dx * std::sin(odometry_.theta) + dy * std::cos(odometry_.theta);
    odometry_.theta += dtheta;
  }
  else
  {
    // Exact integration along the arc the robot drives while turning
    const double theta_old = odometry_.theta;
    odometry_.theta += dtheta;
    odometry_.x += (dx / dtheta) * (std::sin(odometry_.theta) - std::sin(theta_old)) +
                   (dy / dtheta) * (std::cos(odometry_.theta) - std::cos(theta_old));
    odometry_.y += -(dx / dtheta) * (std::cos(odometry_.theta) - std::cos(theta_old)) +
                   (dy / dtheta) * (std::sin(odometry_.theta) - std::sin(theta_old));
  }

  return odometry_;
}
}  // namespace swerve_drive_controller
