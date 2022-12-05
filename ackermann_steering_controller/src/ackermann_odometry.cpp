/*********************************************************************
* Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 *********************************************************************/

/*
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

#include "ackermann_odometry/ackermann_odometry.hpp"

#include <cmath>
#include <iostream>

namespace ackermann_odometry
{
namespace bacc = boost::accumulators;
// using namespace ackermann_steering_controller_ros2;
// ackermann_steering_controller_ros2::Params params;

AckermannOdometry::AckermannOdometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheel_radius_(0.0),
  rear_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(0.0),
  linear_acc_(RollingWindow::window_size = velocity_rolling_window_size),
  angular_acc_(RollingWindow::window_size = velocity_rolling_window_size),
  integrate_fun_(std::bind(
    &AckermannOdometry::integrate_exact, this, std::placeholders::_1, std::placeholders::_2))

{
}

void AckermannOdometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  reset_accumulators();
  timestamp_ = time;
}

// TODO(destogl): enable also velocity interface to update using velocity from the rear wheel
bool AckermannOdometry::update_from_position(
  const double rear_wheel_pos, const double front_steer_pos, const double dt)
{
  /// Get current wheel joint positions:
  const double rear_wheel_cur_pos = rear_wheel_pos * wheel_radius_;

  /// Estimate velocity of wheels using old and current position:
  //const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
  //const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

  const double rear_wheel_est_pos_diff = rear_wheel_cur_pos - rear_wheel_old_pos_;

  /// Update old position with current:
  rear_wheel_old_pos_ = rear_wheel_cur_pos;

  /// Compute linear and angular diff:
  const double linear_velocity =
    rear_wheel_est_pos_diff / dt;  //(right_wheel_est_vel + left_wheel_est_vel) * 0.5;

  //const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_w_;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  /// Integrate odometry:
  integrate_fun_(linear_velocity * dt, angular);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_(linear_velocity);
  angular_acc_(angular / dt);

  linear_ = bacc::rolling_mean(linear_acc_);
  angular_ = bacc::rolling_mean(angular_acc_);

  return true;
}

bool AckermannOdometry::update_from_velocity(
  const double rear_wheel_vel, const double front_steer_pos, const double dt)
{
  // (right_wheel_est_vel + left_wheel_est_vel) * 0.5;
  double linear_velocity = rear_wheel_vel * wheel_radius_;

  //const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_w_;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  /// Integrate odometry:
  integrate_fun_(linear_velocity * dt, angular * dt);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_(linear_velocity);
  angular_acc_(angular);

  linear_ = bacc::rolling_mean(linear_acc_);
  angular_ = bacc::rolling_mean(angular_acc_);

  return true;
}

void AckermannOdometry::update_open_loop(const double linear, const double angular, const double dt)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  integrate_fun_(linear * dt, angular * dt);
}

void AckermannOdometry::set_wheel_params(double wheel_separation, double wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheel_radius_ = wheel_radius;
}

void AckermannOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  reset_accumulators();
}

void AckermannOdometry::integrate_runge_kutta_2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

/**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
void AckermannOdometry::integrate_exact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
    integrate_runge_kutta_2(linear, angular);
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void AckermannOdometry::reset_accumulators()
{
  linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

}  // namespace ackermann_odometry
