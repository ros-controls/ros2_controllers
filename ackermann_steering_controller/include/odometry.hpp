/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 * Author: Masaru Morita
 */

#pragma once

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

//#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/function.hpp>

namespace ros2_ackermann_cont
{

namespace bacc = boost::accumulators;

/**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
class Odometry
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
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /**
     * \brief Initialize the odometry
     * \param time Current time
     */
  void init(const rclcpp::Time & time);

  /**
     * \brief Updates the odometry class with latest wheels position
     * \param rear_wheel_pos  Rear wheel position [rad]
     * \param front_steer_pos Front Steer position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
  bool update(double rear_wheel_pos, double front_steer_pos, const rclcpp::Time & time);

  /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);

  /**
     * \brief heading getter
     * \return heading [rad]
     */
  double getHeading() const { return heading_; }

  /**
     * \brief x position getter
     * \return x position [m]
     */
  double getX() const { return x_; }

  /**
     * \brief y position getter
     * \return y position [m]
     */
  double getY() const { return y_; }

  /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
  double getLinear() const { return linear_; }

  /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
  double getAngular() const { return angular_; }

  /**
     * \brief Sets the wheel parameters: radius and separation
     */
  void setWheelParams(double wheel_reparation_h, double wheel_radius);

  /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  /// Rolling mean accumulator and window:
  typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
  typedef bacc::tag::rolling_window RollingWindow;

  /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
  void integrateRungeKutta2(double linear, double angular);

  /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
  void integrateExact(double linear, double angular);

  /**
     *  \brief Reset linear and angular accumulators
     */
  void resetAccumulators();

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
  double wheel_radius_;

  /// Previous wheel position/state [rad]:
  double rear_wheel_old_pos_;

  /// Rolling mean accumulators for the linar and angular velocities:
  int velocity_rolling_window_size_;
  RollingMeanAcc linear_acc_;
  RollingMeanAcc angular_acc_;

  /// Integration funcion, used to integrate the odometry:
  IntegrationFunction integrate_fun_;
};
}
