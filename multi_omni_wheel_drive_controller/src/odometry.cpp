// Copyright 2025 Aarav Gupta
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

#include "multi_omni_wheel_drive_controller/odometry.hpp"

#include <cmath>

namespace multi_omni_wheel_drive_controller
{
Odometry::Odometry()
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_vel_(0.0),
  linear_y_vel_(0.0),
  angular_vel_(0.0),
  robot_radius_(0.0),
  wheel_radius_(0.0),
  wheels_old_pos_(0.0)
{
}

bool Odometry::updateFromPos(const std::vector<double> & wheels_pos, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  // We cannot estimate angular velocity with very small time intervals
  if (std::fabs(dt) < 1e-6)
  {
    return false;
  }

  // Estimate angular velocity of wheels using old and current position [rads/s]:
  std::vector<double> wheels_vel(wheels_pos.size());
  for (size_t i = 0; i < static_cast<size_t>(wheels_pos.size()); ++i)
  {
    wheels_vel[i] = (wheels_pos[i] - wheels_old_pos_[i]) / dt;
    wheels_old_pos_[i] = wheels_pos[i];
  }

  if (updateFromVel(wheels_vel, time))
  {
    return true;
  }
  return false;
}

bool Odometry::updateFromVel(const std::vector<double> & wheels_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular velocities of the robot:
  const Eigen::Vector3d robot_velocity = compute_robot_velocity(wheels_vel);

  // Integrate odometry:
  integrate(robot_velocity(0) * dt, robot_velocity(1) * dt, robot_velocity(2) * dt);

  timestamp_ = time;

  linear_x_vel_ = robot_velocity(0);
  linear_y_vel_ = robot_velocity(1);
  angular_vel_ = robot_velocity(2);

  return true;
}

Eigen::Vector3d Odometry::compute_robot_velocity(const std::vector<double> & wheels_vel) const
{
  Eigen::MatrixXd A(wheels_vel.size(), 3);   // Transformation Matrix
  Eigen::VectorXd omega(wheels_vel.size());  // Wheel angular velocities vector

  const double angle_bw_wheels = (2 * M_PI) / static_cast<double>(wheels_vel.size());

  for (size_t i = 0; i < wheels_vel.size(); ++i)
  {
    // Define the transformation matrix
    const double theta = (angle_bw_wheels * static_cast<double>(i)) + wheel_offset_;
    A(static_cast<int>(i), 0) = std::sin(theta);
    A(static_cast<int>(i), 1) = -std::cos(theta);
    A(static_cast<int>(i), 2) = -robot_radius_;

    // Define the wheel angular velocities vector
    omega(static_cast<int>(i)) = wheels_vel[i];
  }

  // Compute the robot velocities using SVD decomposition
  const Eigen::Vector3d V =
    A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(omega * wheel_radius_);

  return V;
}

bool Odometry::updateOpenLoop(
  const double & linear_x_vel, const double & linear_y_vel, const double & angular_vel,
  const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Integrate odometry:
  integrate(linear_x_vel * dt, linear_y_vel * dt, angular_vel * dt);

  timestamp_ = time;

  // Save last linear and angular velocity:
  linear_x_vel_ = linear_x_vel;
  linear_y_vel_ = linear_y_vel;
  angular_vel_ = angular_vel;

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setParams(
  const double & robot_radius, const double & wheel_radius, const double & wheel_offset,
  const size_t & wheel_count)
{
  robot_radius_ = robot_radius;
  wheel_radius_ = wheel_radius;
  wheel_offset_ = wheel_offset;
  wheels_old_pos_.resize(wheel_count, 0.0);
}

void Odometry::integrate(const double & dx, const double & dy, const double & dheading)
{
  if (std::fabs(dheading) < 1e-6)
  {
    // For very small dheading, approximate to linear motion
    x_ = x_ + ((dx * std::cos(heading_)) - (dy * std::sin(heading_)));
    y_ = y_ + ((dx * std::sin(heading_)) + (dy * std::cos(heading_)));
    heading_ = heading_ + dheading;
  }
  else
  {
    const double heading_old = heading_;
    heading_ = heading_ + dheading;
    x_ = x_ + ((dx / dheading) * (std::sin(heading_) - std::sin(heading_old))) +
         ((dy / dheading) * (std::cos(heading_) - std::cos(heading_old)));
    y_ = y_ - (dx / dheading) * (std::cos(heading_) - std::cos(heading_old)) +
         (dy / dheading) * (std::sin(heading_) - std::sin(heading_old));
  }
}

}  // namespace multi_omni_wheel_drive_controller
