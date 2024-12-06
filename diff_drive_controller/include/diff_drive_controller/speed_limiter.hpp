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

#ifndef DIFF_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_
#define DIFF_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_

#include <limits>

#include "control_toolbox/rate_limiter.hpp"

namespace diff_drive_controller
{
class SpeedLimiter
{
public:
  /**
   * \brief Constructor
   * \param [in] has_velocity_limits     if true, applies velocity limits
   * \param [in] has_acceleration_limits if true, applies acceleration limits
   * \param [in] has_jerk_limits         if true, applies jerk limits
   * \param [in] min_velocity Minimum velocity [m/s], usually <= 0
   * \param [in] max_velocity Maximum velocity [m/s], usually >= 0
   * \param [in] max_deceleration Maximum deceleration [m/s^2], usually <= 0
   * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
   * \param [in] min_jerk Minimum jerk [m/s^3], usually <= 0
   * \param [in] max_jerk Maximum jerk [m/s^3], usually >= 0
   */
  [[deprecated]] SpeedLimiter(
    bool has_velocity_limits = true, bool has_acceleration_limits = true,
    bool has_jerk_limits = true, double min_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_deceleration = std::numeric_limits<double>::quiet_NaN(),
    double max_acceleration = std::numeric_limits<double>::quiet_NaN(),
    double min_jerk = std::numeric_limits<double>::quiet_NaN(),
    double max_jerk = std::numeric_limits<double>::quiet_NaN())
  {
    if (!has_velocity_limits)
    {
      min_velocity = max_velocity = std::numeric_limits<double>::quiet_NaN();
    }
    if (!has_acceleration_limits)
    {
      max_deceleration = max_acceleration = std::numeric_limits<double>::quiet_NaN();
    }
    if (!has_jerk_limits)
    {
      min_jerk = max_jerk = std::numeric_limits<double>::quiet_NaN();
    }
    speed_limiter_ = control_toolbox::RateLimiter<double>(
      min_velocity, max_velocity, max_deceleration, max_acceleration,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), min_jerk,
      max_jerk);
  }

  /**
   * \brief Constructor
   * \param [in] min_velocity Minimum velocity [m/s], usually <= 0
   * \param [in] max_velocity Maximum velocity [m/s], usually >= 0
   * \param [in] max_acceleration_reverse Maximum acceleration in reverse direction [m/s^2], usually
   * <= 0
   * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
   * \param [in] max_deceleration Maximum deceleration [m/s^2], usually <= 0
   * \param [in] max_deceleration_reverse Maximum deceleration in reverse direction [m/s^2], usually
   * >= 0
   * \param [in] min_jerk Minimum jerk [m/s^3], usually <= 0
   * \param [in] max_jerk Maximum jerk [m/s^3], usually >= 0
   */
  SpeedLimiter(
    double min_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_velocity = std::numeric_limits<double>::quiet_NaN(),
    double max_acceleration_reverse = std::numeric_limits<double>::quiet_NaN(),
    double max_acceleration = std::numeric_limits<double>::quiet_NaN(),
    double max_deceleration = std::numeric_limits<double>::quiet_NaN(),
    double max_deceleration_reverse = std::numeric_limits<double>::quiet_NaN(),
    double min_jerk = std::numeric_limits<double>::quiet_NaN(),
    double max_jerk = std::numeric_limits<double>::quiet_NaN())
  {
    speed_limiter_ = control_toolbox::RateLimiter<double>(
      min_velocity, max_velocity, max_acceleration_reverse, max_acceleration, max_deceleration,
      max_deceleration_reverse, min_jerk, max_jerk);
  }

  /**
   * \brief Limit the velocity and acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt)
  {
    return speed_limiter_.limit(v, v0, v1, dt);
  }

  /**
   * \brief Limit the velocity
   * \param [in, out] v Velocity [m/s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_velocity(double & v) { return speed_limiter_.limit_value(v); }

  /**
   * \brief Limit the acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double & v, double v0, double dt)
  {
    return speed_limiter_.limit_first_derivative(v, v0, dt);
  }

  /**
   * \brief Limit the jerk
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
   */
  double limit_jerk(double & v, double v0, double v1, double dt)
  {
    return speed_limiter_.limit_second_derivative(v, v0, v1, dt);
  }

private:
  control_toolbox::RateLimiter<double> speed_limiter_;  // Instance of the new RateLimiter
};

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_
