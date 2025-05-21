// Copyright 2022 Pixel Robotics.
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
 * Author: Tony Najjar
 */

#ifndef TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_
#define TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_

#include <cmath>

namespace tricycle_controller
{
class TractionLimiter
{
public:
  /**
   * \brief Constructor
   *
   * Parameters are applied symmetrically for both directions, i.e., are applied
   * to the absolute value of the corresponding quantity.
   *
   * \warning
   * - Setting min_velocity: the robot can't stand still
   *
   * - Setting min_deceleration/min_acceleration: the robot can't move with constant velocity
   *
   * - Setting min_jerk: the robot can't move with constant acceleration
   *
   * \param [in] min_velocity Minimum velocity [m/s] or [rad/s]
   * \param [in] max_velocity Maximum velocity [m/s] or [rad/s]
   * \param [in] min_acceleration Minimum acceleration [m/s^2] or [rad/s^2]
   * \param [in] max_acceleration Maximum acceleration [m/s^2] or [rad/s^2]
   * \param [in] min_deceleration Minimum deceleration [m/s^2] or [rad/s^2]
   * \param [in] max_deceleration Maximum deceleration [m/s^2] or [rad/s^2]
   * \param [in] min_jerk Minimum jerk [m/s^3]
   * \param [in] max_jerk Maximum jerk [m/s^3]
   */
  TractionLimiter(
    double min_velocity = NAN, double max_velocity = NAN, double min_acceleration = NAN,
    double max_acceleration = NAN, double min_deceleration = NAN, double max_deceleration = NAN,
    double min_jerk = NAN, double max_jerk = NAN);

  /**
   * \brief Limit the velocity and acceleration
   * \param [in, out] v  Velocity [m/s] or [rad/s]
   * \param [in]      v0 Previous velocity to v  [m/s] or [rad/s]
   * \param [in]      v1 Previous velocity to v0 [m/s] or [rad/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  /**
   * \brief Limit the velocity
   * \param [in, out] v Velocity [m/s] or [rad/s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_velocity(double & v);

  /**
   * \brief Limit the acceleration
   * \param [in, out] v  Velocity [m/s] or [rad/s]
   * \param [in]      v0 Previous velocity [m/s] or [rad/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double & v, double v0, double dt);

  /**
   * \brief Limit the jerk
   * \param [in, out] v  Velocity [m/s] or [rad/s]
   * \param [in]      v0 Previous velocity to v  [m/s] or [rad/s]
   * \param [in]      v1 Previous velocity to v0 [m/s] or [rad/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
   */
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  // Velocity limits:
  double min_velocity_;
  double max_velocity_;

  // Acceleration limits:
  double min_acceleration_;
  double max_acceleration_;

  // Deceleration limits:
  double min_deceleration_;
  double max_deceleration_;

  // Jerk limits:
  double min_jerk_;
  double max_jerk_;
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__TRACTION_LIMITER_HPP_
