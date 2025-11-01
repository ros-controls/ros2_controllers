// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 */

#include <cmath>

#include "diff_drive_controller/odometry.hpp"

namespace diff_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  left_wheel_old_pos_(0.0),
  right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double left_pos, double right_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
  const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_old_pos_;
  const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

  // Update old position with current:
  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);

  return true;
}

bool Odometry::updateFromPos(const double left_pos, const double right_pos, const double dt)
{
  // We cannot estimate angular velocity with very small time intervals
  if (std::fabs(dt) < 1e-6)
  {
    return false;
  }

  // Estimate angular velocity of wheels using old and current position [rads/s]:
  double left_vel = (left_pos - left_wheel_old_pos_) / dt;
  double right_vel = (right_pos - right_wheel_old_pos_) / dt;

  // Update old position with current:
  left_wheel_old_pos_ = left_pos;
  right_wheel_old_pos_ = right_pos;

  return updateFromVel(left_vel, right_vel, dt);
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }
  // Compute linear and angular diff:
  const double linear = (left_vel + right_vel) * 0.5;
  // Now there is a bug about scout angular velocity
  const double angular = (right_vel - left_vel) / wheel_separation_;

  // Integrate odometry:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

bool Odometry::updateFromVel(const double left_vel, const double right_vel, const double dt)
{
  // Compute linear and angular velocities of the robot:
  const double linear_vel = (left_vel * left_wheel_radius_ + right_vel * right_wheel_radius_) * 0.5;
  const double angular_vel =
    (right_vel * right_wheel_radius_ - left_vel * left_wheel_radius_) / wheel_separation_;

  // Integrate odometry:
  integrate(linear_vel * dt, angular_vel * dt);

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear_vel);
  angular_accumulator_.accumulate(angular_vel);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

bool Odometry::tryUpdateOpenLoop(const double linear_vel, const double angular_vel, const double dt)
{
  // Integrate odometry:
  integrate(linear_vel * dt, angular_vel * dt);

  // Save last linear and angular velocity:
  linear_ = linear_vel;
  angular_ = angular_vel;

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double left_wheel_radius, double right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * std::cos(direction);
  y_ += linear * std::sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (std::sin(heading_) - std::sin(heading_old));
    y_ += -r * (std::cos(heading_) - std::cos(heading_old));
  }
}

void Odometry::integrate(const double dx, const double dheading)
{
  if (fabs(dheading) < 1e-6)
  {
    // For very small dheading, approximate to linear motion
    x_ += (dx * std::cos(heading_));
    y_ += (dx * std::sin(heading_));
    heading_ += dheading;
  }
  else
  {
    const double heading_old = heading_;
    heading_ += dheading;
    x_ += ((dx / dheading) * (std::sin(heading_) - std::sin(heading_old)));
    y_ += -(dx / dheading) * (std::cos(heading_) - std::cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller
