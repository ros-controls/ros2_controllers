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

#include <algorithm>
#include <stdexcept>
#include <string>

#include "tricycle_controller/steering_limiter.hpp"

namespace tricycle_controller
{
SteeringLimiter::SteeringLimiter(
  double min_position, double max_position, double min_velocity, double max_velocity,
  double min_acceleration, double max_acceleration, double min_deceleration, double max_deceleration)
: min_position_(min_position),
  max_position_(max_position),
  min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration),
  min_deceleration_(min_deceleration),
  max_deceleration_(max_deceleration)
{
  if (!std::isnan(min_position_) && std::isnan(max_position_)) max_position_ = -min_position_;
  if (!std::isnan(max_position_) && std::isnan(min_position_)) min_position_ = -max_position_;

  if (!std::isnan(min_velocity_) && std::isnan(max_velocity_))
    max_velocity_ = 1000.0;  // Arbitrarily big number
  if (!std::isnan(max_velocity_) && std::isnan(min_velocity_)) min_velocity_ = 0.0;

  if (!std::isnan(min_acceleration_) && std::isnan(max_acceleration_)) max_acceleration_ = 1000.0;
  if (!std::isnan(max_acceleration_) && std::isnan(min_acceleration_)) min_acceleration_ = 0.0;

  if (!std::isnan(min_deceleration_) && std::isnan(max_deceleration_)) max_deceleration_ = 1000.0;
  if (!std::isnan(max_deceleration_) && std::isnan(min_deceleration_)) min_deceleration_ = 0.0;

  const std::string error =
    "The positive limit will be applied to both directions. Setting different limits for positive "
    "and negative directions is not supported. Actuators are "
    "assumed to have the same constraints in both directions";

  if (min_velocity_ < 0 || max_velocity_ < 0)
  {
    throw std::invalid_argument("Velocity cannot be negative." + error);
  }

  if (min_acceleration_ < 0 || max_acceleration_ < 0)
  {
    throw std::invalid_argument("Acceleration cannot be negative." + error);
  }

  if (min_deceleration_ < 0 || max_deceleration_ < 0)
  {
    throw std::invalid_argument("Deceleration cannot be negative." + error);
  }
}

double SteeringLimiter::limit(double & p, double p0, double p1, double dt)
{
  const double tmp = p;

  if (!std::isnan(min_acceleration_) && !std::isnan(max_acceleration_))
    limit_acceleration(p, p0, p1, dt);
  if (!std::isnan(min_velocity_) && !std::isnan(max_velocity_)) limit_velocity(p, p0, dt);
  if (!std::isnan(min_position_) && !std::isnan(max_position_)) limit_position(p);

  return tmp != 0.0 ? p / tmp : 1.0;
}

double SteeringLimiter::limit_position(double & p)
{
  const double tmp = p;
  p = std::clamp(p, min_position_, max_position_);

  return tmp != 0.0 ? p / tmp : 1.0;
}

double SteeringLimiter::limit_velocity(double & p, double p0, double dt)
{
  const double tmp = p;

  const double dv_min = min_velocity_ * dt;
  const double dv_max = max_velocity_ * dt;

  double dv = std::clamp(std::fabs(p - p0), dv_min, dv_max);
  dv *= (p - p0 >= 0 ? 1 : -1);
  p = p0 + dv;

  return tmp != 0.0 ? p / tmp : 1.0;
}

double SteeringLimiter::limit_acceleration(double & p, double p0, double p1, double dt)
{
  const double tmp = p;

  const double dv = p - p0;
  const double dp0 = p0 - p1;

  const double dt2 = 2. * dt * dt;

  double da_min;
  double da_max;
  
  // Determine if we're accelerating or decelerating based on velocity magnitude change
  if (std::fabs(dv) >= std::fabs(dp0))
  {
    // Accelerating (increasing speed)
    da_min = min_acceleration_ * dt2;
    da_max = max_acceleration_ * dt2;
  }
  else
  {
    // Decelerating (decreasing speed) - use deceleration limits if available
    if (!std::isnan(min_deceleration_) && !std::isnan(max_deceleration_))
    {
      da_min = min_deceleration_ * dt2;
      da_max = max_deceleration_ * dt2;
    }
    else
    {
      // Fall back to acceleration limits if deceleration limits not set
      da_min = min_acceleration_ * dt2;
      da_max = max_acceleration_ * dt2;
    }
  }

  double da = std::clamp(std::fabs(dv - dp0), da_min, da_max);
  da *= (dv - dp0 >= 0 ? 1 : -1);
  p = p0 + dp0 + da;

  return tmp != 0.0 ? p / tmp : 1.0;
}

}  // namespace tricycle_controller
