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
 * Author: Enrique Fernández
 */

#ifndef DIFF_DRIVE_CONTROLLER__OSPEED_LIMITER_HPP_
#define DIFF_DRIVE_CONTROLLER__OSPEED_LIMITER_HPP_

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
   * \param [in] min_acceleration Minimum acceleration [m/s^2], usually <= 0
   * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
   * \param [in] min_jerk Minimum jerk [m/s^3], usually <= 0
   * \param [in] max_jerk Maximum jerk [m/s^3], usually >= 0
   */
  SpeedLimiter(
    bool has_velocity_limits = false, bool has_acceleration_limits = false,
    bool has_jerk_limits = false,
    double min_velocity = 0.0, double max_velocity = 0.0, double min_acceleration = 0.0,
    double max_acceleration = 0.0, double min_jerk = 0.0, double max_jerk = 0.0);

  /**
   * \brief Limit the velocity and acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  /**
   * \brief Limit the velocity
   * \param [in, out] v Velocity [m/s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_velocity(double & v);

  /**
   * \brief Limit the acceleration
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double & v, double v0, double dt);

  /**
   * \brief Limit the jerk
   * \param [in, out] v  Velocity [m/s]
   * \param [in]      v0 Previous velocity to v  [m/s]
   * \param [in]      v1 Previous velocity to v0 [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
   */
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  // Enable/Disable velocity/acceleration/jerk limits:
  bool has_velocity_limits_;
  bool has_acceleration_limits_;
  bool has_jerk_limits_;

  // Velocity limits:
  double min_velocity_;
  double max_velocity_;

  // Acceleration limits:
  double min_acceleration_;
  double max_acceleration_;

  // Jerk limits:
  double min_jerk_;
  double max_jerk_;
};

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER__OSPEED_LIMITER_HPP_
