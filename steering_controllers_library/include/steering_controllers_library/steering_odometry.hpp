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
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

#ifdef _WIN32
#pragma message( \
  "This header is deprecated. Please update your code to use 'steering_kinematics.hpp' header.")  // NOLINT
#else
#warning \
  "This header is deprecated. Please update your code to use 'steering_kinematics.hpp' header." //NOLINT
#endif

#include "steering_controllers_library/steering_kinematics.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace steering_odometry
{
[[deprecated("Use steering_kinematics::BICYCLE_CONFIG")]] const unsigned int BICYCLE_CONFIG =
  steering_kinematics::BICYCLE_CONFIG;
[[deprecated("Use steering_kinematics::TRICYCLE_CONFIG")]] const unsigned int TRICYCLE_CONFIG =
  steering_kinematics::TRICYCLE_CONFIG;
[[deprecated("Use steering_kinematics::ACKERMANN_CONFIG")]] const unsigned int ACKERMANN_CONFIG =
  steering_kinematics::ACKERMANN_CONFIG;

inline bool is_close_to_zero(double val) { return steering_kinematics::is_close_to_zero(val); }

/**
 * \brief Deprecated Odometry class for backward ABI compatibility.
 * Internally calling steering_kinematics::SteeringKinematics
 */

class [[deprecated("Use steering_kinematics::SteeringKinematics instead")]] SteeringOdometry
{
public:
  explicit SteeringOdometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);

  bool update_from_position(
    const double traction_wheel_pos, const double steer_pos, const double dt);

  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double steer_pos, const double dt);

  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  bool update_from_velocity(
    const double traction_wheel_vel, const double steer_pos, const double dt);

  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos, const double dt);

  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  void update_open_loop(const double v_bx, const double omega_bz, const double dt);

  void set_odometry_type(const unsigned int type);

  unsigned int get_odometry_type() const;
  double get_heading() const;

  double get_x() const;

  double get_y() const;

  double get_linear() const;

  double get_angular() const;

  void set_wheel_params(
    const double wheel_radius, const double wheel_base, const double wheel_track);

  void set_wheel_params(
    const double wheel_radius, const double wheel_base, const double wheel_track_steering,
    const double wheel_track_traction);

  void set_velocity_rolling_window_size(const size_t velocity_rolling_window_size);

  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    double v_bx, double omega_bz, bool open_loop = true,
    bool reduce_wheel_speed_until_steering_reached = false);

  void reset_odometry();

  bool try_update_open_loop(double linear, double angular, double delTime);

  bool try_update_from_position(double right_traction, double left_traction, double right_steering, double left_steering, double delTime);

  bool try_update_from_velocity(double right_traction, double left_traction, double right_steering, double left_steering, double delTime);

private:
  steering_kinematics::SteeringKinematics sk_impl_;
};
}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
