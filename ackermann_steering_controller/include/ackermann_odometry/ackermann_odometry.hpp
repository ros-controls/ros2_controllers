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

#pragma once

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

//#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/function.hpp>

namespace ackermann_odometry
{
namespace bacc = boost::accumulators;

/**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
class AckermannOdometry
{
public:
  /// Integration function, used to integrate the odometry:
  typedef boost::function<void(double, double)> IntegrationFunction;

  /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     *
     */
  // ackermann_steering_controller_ros2::Params params;
  explicit AckermannOdometry(size_t velocity_rolling_window_size = 10);

  /**
     * \brief Initialize the odometry
     * \param time Current time
     */
  void init(const rclcpp::Time & time);

  /**
     * \brief Updates the odometry class with latest wheels position
     * \param rear_wheel_pos  Rear wheel position [rad]
     * \param front_steer_pos Front Steer position [rad]
     * \param dt      time difference to last call
     * \return true if the odometry is actually updated
     */
  bool update_from_position(
    const double rear_wheel_pos, const double front_steer_pos, const double dt);

  /**
     * \brief Updates the odometry class with latest wheels position
     * \param rear_wheel_vel  Rear wheel velocity [rad/s]
     * \param front_steer_pos Front Steer position [rad]
     * \param dt      time difference to last call
     * \return true if the odometry is actually updated
     */
  bool update_from_velocity(
    const double rear_wheel_vel, const double front_steer_pos, const double dt);

  /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
  void update_open_loop(const double linear, const double angular, const double dt);

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
     * \brief Sets the wheel parameters: radius and separation
     */
  void set_wheel_params(double wheel_reparation_h, double wheel_radius, double wheelbase);

  /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
  void set_velocity_rolling_window_size(size_t velocity_rolling_window_size);

  /**
    * \brief TODO
    * \param Vx  TODO
    * \param theta_dot TODO
    * \param wheelbase TODO
    */
  double convert_trans_rot_vel_to_steering_angle(double Vx, double theta_dot, double wheelbase);

  /**
     * \brief TODO
     * \param Vx  TODO
     * \param theta_dot TODO
     */
  std::tuple<double, double> twist_to_ackermann(double Vx, double theta_dot);

private:
  /// Rolling mean accumulator and window:
  typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
  typedef bacc::tag::rolling_window RollingWindow;

  /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
  void integrate_runge_kutta_2(double linear, double angular);

  /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
  void integrate_exact(double linear, double angular);

  /**
     *  \brief Reset linear and angular accumulators
     */
  void reset_accumulators();

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  /// Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  double wheel_separation_;
  double wheelbase_;
  double wheel_radius_;

  /// Previous wheel position/state [rad]:
  double rear_wheel_old_pos_;

  /// Rolling mean accumulators for the linar and angular velocities:
  int velocity_rolling_window_size_;
  RollingMeanAcc linear_acc_;
  RollingMeanAcc angular_acc_;

  /// Integration function, used to integrate the odometry:
  IntegrationFunction integrate_fun_;
};
}  // namespace ackermann_odometry
