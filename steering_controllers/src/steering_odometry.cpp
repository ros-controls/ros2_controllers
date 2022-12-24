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

#include "steering_controllers/steering_odometry.hpp"

#include <cmath>
#include <iostream>

namespace steering_odometry
{
SteeringOdometry::SteeringOdometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  rear_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringOdometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  reset_accumulators();
  timestamp_ = time;
}

bool SteeringOdometry::update_odometry(
  const double linear_velocity, const double angular, const double dt)
{
  /// Integrate odometry:
  SteeringOdometry::integrate_exact(linear_velocity * dt, angular);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_.accumulate(linear_velocity);
  angular_acc_.accumulate(angular / dt);

  linear_ = linear_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

// TODO(petkovich): enable also velocity interface to update using velocity from the rear wheel
bool SteeringOdometry::update_from_position(
  const double rear_wheel_pos, const double front_steer_pos, const double dt)
{
  /// Get current wheel joint positions:
  const double rear_wheel_cur_pos = rear_wheel_pos * wheel_radius_;
  const double rear_wheel_est_pos_diff = rear_wheel_cur_pos - rear_wheel_old_pos_;

  /// Update old position with current:
  rear_wheel_old_pos_ = rear_wheel_cur_pos;

  /// Compute linear and angular diff:
  const double linear_velocity = rear_wheel_est_pos_diff / dt;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

bool SteeringOdometry::update_from_position(
  const double rear_right_wheel_pos, const double rear_left_wheel_pos, const double front_steer_pos,
  const double dt)
{
  /// Get current wheel joint positions:
  const double rear_right_wheel_cur_pos = rear_right_wheel_pos * wheel_radius_;
  const double rear_left_wheel_cur_pos = rear_left_wheel_pos * wheel_radius_;

  const double rear_right_wheel_est_pos_diff = rear_right_wheel_cur_pos - rear_right_wheel_old_pos_;
  const double rear_left_wheel_est_pos_diff = rear_left_wheel_cur_pos - rear_left_wheel_old_pos_;

  /// Update old position with current:
  rear_right_wheel_old_pos_ = rear_right_wheel_cur_pos;
  rear_left_wheel_old_pos_ = rear_left_wheel_cur_pos;

  const double linear_velocity =
    (rear_right_wheel_est_pos_diff + rear_left_wheel_est_pos_diff) * 0.5 / dt;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

bool SteeringOdometry::update_from_position(
  const double rear_right_wheel_pos, const double rear_left_wheel_pos,
  const double front_right_steer_pos, const double front_left_steer_pos, const double dt)
{
  /// Get current wheel joint positions:
  const double rear_right_wheel_cur_pos = rear_right_wheel_pos * wheel_radius_;
  const double rear_left_wheel_cur_pos = rear_left_wheel_pos * wheel_radius_;

  const double rear_right_wheel_est_pos_diff = rear_right_wheel_cur_pos - rear_right_wheel_old_pos_;
  const double rear_left_wheel_est_pos_diff = rear_left_wheel_cur_pos - rear_left_wheel_old_pos_;

  /// Update old position with current:
  rear_right_wheel_old_pos_ = rear_right_wheel_cur_pos;
  rear_left_wheel_old_pos_ = rear_left_wheel_cur_pos;

  /// Compute linear and angular diff:
  const double linear_velocity =
    (rear_right_wheel_est_pos_diff + rear_left_wheel_est_pos_diff) * 0.5 / dt;
  const double front_steer_pos = (front_right_steer_pos + front_left_steer_pos) * 0.5;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

bool SteeringOdometry::update_from_velocity(
  const double rear_wheel_vel, const double front_steer_pos, const double dt)
{
  double linear_velocity = rear_wheel_vel * wheel_radius_;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

bool SteeringOdometry::update_from_velocity(
  const double rear_right_wheel_vel, const double rear_left_wheel_vel, const double front_steer_pos,
  const double dt)
{
  double linear_velocity = (rear_right_wheel_vel + rear_left_wheel_vel) * wheel_radius_ * 0.5;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

bool SteeringOdometry::update_from_velocity(
  const double rear_right_wheel_vel, const double rear_left_wheel_vel,
  const double front_right_steer_pos, const double front_left_steer_pos, const double dt)
{
  double linear_velocity = (rear_right_wheel_vel + rear_left_wheel_vel) * wheel_radius_ * 0.5;
  const double front_steer_pos = (front_right_steer_pos + front_left_steer_pos) * 0.5;
  const double angular = tan(front_steer_pos) * linear_velocity / wheel_separation_;

  update_odometry(linear_velocity, angular, dt);

  return true;
}

void SteeringOdometry::update_open_loop(const double linear, const double angular, const double dt)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  SteeringOdometry::integrate_exact(linear * dt, angular * dt);
}

void SteeringOdometry::set_wheel_params(
  double wheel_radius, double wheel_separation, double wheelbase)
{
  wheel_radius_ = wheel_radius;
  wheel_separation_ = wheel_separation;
  wheelbase_ = wheelbase;
}

void SteeringOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  reset_accumulators();
}

// TODO(petkovich): change functions depending on fwd kinematics model
double SteeringOdometry::convert_trans_rot_vel_to_steering_angle(double Vx, double theta_dot)
{
  if (theta_dot == 0 || Vx == 0)
  {
    return 0;
  }
  return std::atan(theta_dot * wheel_separation_ / Vx);
}

// TODO(petkovich): change functions depending on fwd kinematics model
std::tuple<double, double> SteeringOdometry::twist_to_ackermann(double Vx, double theta_dot)
{
  // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
  double alpha, Ws;

  if (Vx == 0 && theta_dot != 0)
  {  // is spin action
    alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
    Ws = abs(theta_dot) * wheel_separation_ / wheel_radius_;
    return std::make_tuple(alpha, Ws);
  }

  alpha = SteeringOdometry::convert_trans_rot_vel_to_steering_angle(Vx, theta_dot);
  Ws = Vx / (wheel_radius_ * std::cos(alpha));
  return std::make_tuple(alpha, Ws);
}

void SteeringOdometry::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  reset_accumulators();
}

void SteeringOdometry::integrate_runge_kutta_2(double linear, double angular)
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
void SteeringOdometry::integrate_exact(double linear, double angular)
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

void SteeringOdometry::reset_accumulators()
{
  linear_acc_ = rcpputils::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_acc_ = rcpputils::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  // TODO(petkovich): angular rolling window size?
}
}  // namespace steering_odometry
