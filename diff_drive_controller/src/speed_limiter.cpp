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

#include <algorithm>
#include <stdexcept>

#include "diff_drive_controller/speed_limiter.hpp"
#include "rcppmath/clamp.hpp"

namespace diff_drive_controller
{
SpeedLimiter::SpeedLimiter(
  bool has_velocity_limits, bool has_acceleration_limits, bool has_jerk_limits, double min_velocity,
  double max_velocity, double min_acceleration, double max_acceleration, double min_jerk,
  double max_jerk)
: has_velocity_limits_(has_velocity_limits),
  has_acceleration_limits_(has_acceleration_limits),
  has_jerk_limits_(has_jerk_limits),
  min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration),
  min_jerk_(min_jerk),
  max_jerk_(max_jerk)
{
  // Check if limits are valid, max must be specified, min defaults to -max if unspecified
  if (has_velocity_limits_)
  {
    if (std::isnan(max_velocity_))
    {
      throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
    }
    if (std::isnan(min_velocity_))
    {
      min_velocity_ = -max_velocity_;
    }
  }
  if (has_acceleration_limits_)
  {
    if (std::isnan(max_acceleration_))
    {
      throw std::runtime_error(
        "Cannot apply acceleration limits if max_acceleration is not specified");
    }
    if (std::isnan(min_acceleration_))
    {
      min_acceleration_ = -max_acceleration_;
    }
  }
  if (has_jerk_limits_)
  {
    if (std::isnan(max_jerk_))
    {
      throw std::runtime_error("Cannot apply jerk limits if max_jerk is not specified");
    }
    if (std::isnan(min_jerk_))
    {
      min_jerk_ = -max_jerk_;
    }
  }
}

double SpeedLimiter::limit(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  limit_jerk(v, v0, v1, dt);
  limit_acceleration(v, v0, dt);
  limit_velocity(v);

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_velocity(double & v)
{
  const double tmp = v;

  if (has_velocity_limits_)
  {
    v = rcppmath::clamp(v, min_velocity_, max_velocity_);
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_acceleration(double & v, double v0, double dt)
{
  const double tmp = v;

  if (has_acceleration_limits_)
  {
    const double dv_min = min_acceleration_ * dt;
    const double dv_max = max_acceleration_ * dt;

    const double dv = rcppmath::clamp(v - v0, dv_min, dv_max);

    v = v0 + dv;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_jerk(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  if (has_jerk_limits_)
  {
    const double dv = v - v0;
    const double dv0 = v0 - v1;

    const double dt2 = 2. * dt * dt;

    const double da_min = min_jerk_ * dt2;
    const double da_max = max_jerk_ * dt2;

    const double da = rcppmath::clamp(dv - dv0, da_min, da_max);

    v = v0 + dv0 + da;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

}  // namespace diff_drive_controller
