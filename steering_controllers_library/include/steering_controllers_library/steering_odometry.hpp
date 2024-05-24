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

#include <tuple>
#include <vector>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace steering_odometry
{
const unsigned int BICYCLE_CONFIG = 0;
const unsigned int TRICYCLE_CONFIG = 1;
const unsigned int ACKERMANN_CONFIG = 2;
/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class SteeringOdometry
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   *
   */
  explicit SteeringOdometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param time Current time
   */
  void init(const rclcpp::Time & time);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_pos  traction wheel position [rad]
   * \param steer_pos Front Steer position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double traction_wheel_pos, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel velocity [rad]
   * \param left_traction_wheel_pos  Left traction wheel velocity [rad]
   * \param front_steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel position [rad]
   * \param left_traction_wheel_pos  Left traction wheel position [rad]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_vel  Traction wheel velocity [rad/s]
   * \param front_steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double traction_wheel_vel, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param front_steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest velocity command
   * \param linear  Linear velocity [m/s]
   * \param angular Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  void update_open_loop(const double linear, const double angular, const double dt);

  /**
   * \brief Set odometry type
   * \param type odometry type
   */
  void set_odometry_type(const unsigned int type);

  /**
   * \brief heading getter
   * \return heading [rad]
   */
  double get_heading() const { return heading_; }

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double get_x() const { return x_; }

  /**
   * \brief y position getter
   * \return y position [m]
   */
  double get_y() const { return y_; }

  /**
   * \brief linear velocity getter
   * \return linear velocity [m/s]
   */
  double get_linear() const { return linear_; }

  /**
   * \brief angular velocity getter
   * \return angular velocity [rad/s]
   */
  double get_angular() const { return angular_; }

  /**
   * \brief Sets the wheel parameters: radius, separation and wheelbase
   */
  void set_wheel_params(
    const double wheel_radius, const double wheelbase = 0.0, const double wheel_track = 0.0);

  /**
   * \brief Velocity rolling window size setter
   * \param velocity_rolling_window_size Velocity rolling window size
   */
  void set_velocity_rolling_window_size(const size_t velocity_rolling_window_size);

  /**
   * \brief Calculates inverse kinematics for the desired linear and angular velocities
   * \param v_bx     Linear velocity of the robot in x_b-axis direction
   * \param omega_bz Angular velocity of the robot around x_z-axis
   * \return Tuple of velocity commands and steering commands
   */
  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    const double v_bx, const double omega_bz);

  /**
   *  \brief Reset poses, heading, and accumulators
   */
  void reset_odometry();

private:
  /**
   * \brief Uses precomputed linear and angular velocities to compute odometry
   * \param v_bx  Linear  velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  bool update_odometry(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
   * \param v_bx Linear velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_runge_kutta_2(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular)
   * \param v_bx Linear velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_fk(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Calculates steering angle from the desired translational and rotational velocity
   * \param v_bx     Linear velocity of the robot in x_b-axis direction
   * \param omega_bz Angular velocity of the robot around x_z-axis
   */
  double convert_trans_rot_vel_to_steering_angle(const double v_bx, const double omega_bz);

  /**
   *  \brief Reset linear and angular accumulators
   */
  void reset_accumulators();

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Current pose:
  double x_;          //   [m]
  double y_;          //   [m]
  double steer_pos_;  // [rad]
  double heading_;    // [rad]

  /// Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  /// Kinematic parameters
  double wheel_track_;   // [m]
  double wheelbase_;     // [m]
  double wheel_radius_;  // [m]

  /// Configuration type used for the forward kinematics
  int config_type_ = -1;

  /// Previous wheel position/state [rad]:
  double traction_wheel_old_pos_;
  double traction_right_wheel_old_pos_;
  double traction_left_wheel_old_pos_;
  /// Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_acc_;
  RollingMeanAccumulator angular_acc_;
};
}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
