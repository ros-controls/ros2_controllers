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

bool Odometry::updateFromPosition(double left_pos, double right_pos, const rclcpp::Time & time, const double dt)
{
  // We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
  const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double left_wheel_est_vel = (left_wheel_cur_pos - left_wheel_old_pos_)/dt;
  const double right_wheel_est_vel = (right_wheel_cur_pos - right_wheel_old_pos_)/dt;

  // Update old position with current:
  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time, dt);

  return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time & time, const double dt)
{
  // Compute linear and angular diff:
  const double linear = (left_vel + right_vel) * 0.5;
  // Now there is a bug about scout angular velocity
  const double angular = (right_vel - left_vel) / wheel_separation_;

  // Integrate odometry:
  integrateExact(linear, angular, dt);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear);
  angular_accumulator_.accumulate(angular);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateFromVelocityOpenLoop(double linear, double angular, const rclcpp::Time & time, const double dt)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  timestamp_ = time;
  integrateExact(linear, angular, dt);
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

void Odometry::integrateRungeKutta2(double linear, double angular, const double dt)
{
  const double direction = heading_ + angular * 0.5 * dt;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction) * dt;
  y_ += linear * sin(direction) * dt;
  heading_ += angular * dt;
}

void Odometry::integrateExact(double linear, double angular, const double dt)
{
  if (fabs(angular*dt) < 1e-6)
  {
    integrateRungeKutta2(linear, angular, dt);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular * dt;
    heading_ += angular * dt;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller
