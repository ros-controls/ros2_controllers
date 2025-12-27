// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

#define _USE_MATH_DEFINES

#include "steering_controllers_library/steering_odometry.hpp"

namespace steering_odometry
{
SteeringOdometry::SteeringOdometry(size_t velocity_rolling_window_size)
: sk_impl_(velocity_rolling_window_size)
{
}

void SteeringOdometry::init(const rclcpp::Time & time) { sk_impl_.init(time); }

bool SteeringOdometry::update_from_position(
  const double traction_wheel_pos, const double steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(traction_wheel_pos) || !std::isfinite(steer_pos)) return false;
  return sk_impl_.update_from_position(traction_wheel_pos, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(traction_right_wheel_pos) ||
     !std::isfinite(traction_left_wheel_pos) || !std::isfinite(steer_pos)) return false;
  return sk_impl_.update_from_position(
    traction_right_wheel_pos, traction_left_wheel_pos, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(traction_right_wheel_pos) ||
     !std::isfinite(traction_left_wheel_pos) || !std::isfinite(right_steer_pos) || !std::isfinite(left_steer_pos)) return false;
  return sk_impl_.update_from_position(
    traction_right_wheel_pos, traction_left_wheel_pos, right_steer_pos, left_steer_pos, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double traction_wheel_vel, const double steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(traction_wheel_vel) || !std::isfinite(steer_pos)) return false;
  return sk_impl_.update_from_velocity(traction_wheel_vel, steer_pos, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(right_traction_wheel_vel) ||
     !std::isfinite(left_traction_wheel_vel) || !std::isfinite(steer_pos)) return false;
  return sk_impl_.update_from_velocity(
    right_traction_wheel_vel, left_traction_wheel_vel, steer_pos, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(right_traction_wheel_vel) ||
     !std::isfinite(left_traction_wheel_vel) || !std::isfinite(right_steer_pos) || !std::isfinite(left_steer_pos)) return false;
  return sk_impl_.update_from_velocity(
    right_traction_wheel_vel, left_traction_wheel_vel, right_steer_pos, left_steer_pos, dt);
}

void SteeringOdometry::update_open_loop(const double v_bx, const double omega_bz, const double dt)
{
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(v_bx) || !std::isfinite(omega_bz)) return;
  sk_impl_.update_open_loop(v_bx, omega_bz, dt);
}

bool SteeringOdometry::try_update_open_loop(const double v_bx, const double omega_bz, const double dt){
  if(std::fabs(dt) < std::numeric_limits<double>::epsilon() || !std::isfinite(v_bx) || !std::isfinite(omega_bz)) return false;
  sk_impl_.try_update_open_loop(v_bx, omega_bz, dt);
  return true;
}

void SteeringOdometry::set_wheel_params(double wheel_radius, double wheel_base, double wheel_track)
{
  sk_impl_.set_wheel_params(wheel_radius, wheel_base, wheel_track);
}

void SteeringOdometry::set_wheel_params(
  double wheel_radius, double wheel_base, double wheel_track_steering, double wheel_track_traction)
{
  sk_impl_.set_wheel_params(wheel_radius, wheel_base, wheel_track_steering, wheel_track_traction);
}

void SteeringOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  sk_impl_.set_velocity_rolling_window_size(velocity_rolling_window_size);
}

void SteeringOdometry::set_odometry_type(const unsigned int type)
{
  sk_impl_.set_odometry_type(type);
}

std::tuple<std::vector<double>, std::vector<double>> SteeringOdometry::get_commands(
  const double v_bx, const double omega_bz, const bool open_loop,
  const bool reduce_wheel_speed_until_steering_reached)
{
  return sk_impl_.get_commands(
    v_bx, omega_bz, open_loop, reduce_wheel_speed_until_steering_reached);
}

void SteeringOdometry::reset_odometry() { sk_impl_.reset_odometry(); }
}  // namespace steering_odometry
