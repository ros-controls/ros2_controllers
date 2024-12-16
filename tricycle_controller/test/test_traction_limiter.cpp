// Copyright 2024 AIT - Austrian Institute of Technology GmbH
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

#include <gmock/gmock.h>
#include <limits>

#include "tricycle_controller/traction_limiter.hpp"

TEST(SpeedLimiterTest, testWrongParams)
{
  EXPECT_NO_THROW(tricycle_controller::TractionLimiter limiter(
    std::numeric_limits<double>::quiet_NaN(),  // min_velocity
    std::numeric_limits<double>::quiet_NaN(),  // max_velocity
    std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
    std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
    std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
    std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
    std::numeric_limits<double>::quiet_NaN(),  // min_jerk
    std::numeric_limits<double>::quiet_NaN()   // max_jerk
    ));

  // velocity
  {
    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        -10.,                                      // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        -10.,                                      // min_velocity
        -20.,                                      // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        -10.,                                      // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        20.,                                       // min_velocity
        10.,                                       // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);
  }

  // acceleration
  {
    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        -10.,                                      // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        -10.,                                      // min_acceleration
        -20.,                                      // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        -10.,                                      // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        20.,                                       // min_acceleration
        10.,                                       // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);
  }

  // deceleration
  {
    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        -10.,                                      // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        -10.,                                      // min_deceleration
        -20.,                                      // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        -10.,                                      // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        20.,                                       // min_deceleration
        10.,                                       // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);
  }

  // jerk
  {
    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        -10.,                                      // min_jerk
        std::numeric_limits<double>::quiet_NaN()   // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        -10.,                                      // min_jerk
        -20.                                       // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_jerk
        -10.                                       // max_jerk
        ),
      std::invalid_argument);

    EXPECT_THROW(
      tricycle_controller::TractionLimiter limiter(
        std::numeric_limits<double>::quiet_NaN(),  // min_velocity
        std::numeric_limits<double>::quiet_NaN(),  // max_velocity
        std::numeric_limits<double>::quiet_NaN(),  // min_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_acceleration
        std::numeric_limits<double>::quiet_NaN(),  // min_deceleration
        std::numeric_limits<double>::quiet_NaN(),  // max_deceleration
        20.,                                       // min_jerk
        10.                                        // max_jerk
        ),
      std::invalid_argument);
  }
}

TEST(SpeedLimiterTest, testNoLimits)
{
  tricycle_controller::TractionLimiter limiter;
  double v = 10.0;
  double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
  // check if the velocity is not limited
  EXPECT_DOUBLE_EQ(v, 10.0);
  EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
  v = -10.0;
  limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
  // check if the velocity is not limited
  EXPECT_DOUBLE_EQ(v, -10.0);
  EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
}

TEST(SpeedLimiterTest, testVelocityLimits)
{
  tricycle_controller::TractionLimiter limiter(0.5, 1.0, 0.5, 1.0, 2.0, 3.0, 0.5, 5.0);
  {
    double v = 10.0;
    double limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now 1.0 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, 1.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.1);

    v = 0.1;
    limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now 0.5 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 0.1);

    // TODO(christophfroehlich): does this behavior make sense?
    v = 0.0;
    limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now 0.5 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!

    v = -10.0;
    limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now -1.0 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, -1.0);
    EXPECT_DOUBLE_EQ(limiting_factor, -1.0 / -10.0);

    v = -0.1;
    limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now -0.5 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, -0.5 / -0.1);
  }

  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);
  }
}

TEST(SpeedLimiterTest, testVelocityNoLimits)
{
  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);
    double v = 10.0;
    double limiting_factor = limiter.limit_velocity(v);
    // check if the velocity is not limited
    EXPECT_DOUBLE_EQ(v, 10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);

    v = -10.0;
    limiting_factor = limiter.limit_velocity(v);
    // check if the velocity is not limited
    EXPECT_DOUBLE_EQ(v, -10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
  }

  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);
  }

  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 5.0);
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // jerk is now limiting, not velocity
    EXPECT_DOUBLE_EQ(v, 2.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 2.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // jerk is now limiting, not velocity
    EXPECT_DOUBLE_EQ(v, -2.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 2.5 / 10.0);
  }
}

TEST(SpeedLimiterTest, testAccelerationLimits)
{
  {
    // test max_acceleration
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);

    double v = 10.0;
    double limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now -0.5 m.s-1, which is -1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);
  }
  {
    // test min_acceleration
    // TODO(christophfroehlich): does this behavior make sense?
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    double v = 0.0;
    double limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now 0.25m.s-1 = 0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!

    v = -std::numeric_limits<double>::epsilon();
    limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now -0.25m.s-1 = -0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
  }

  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);

    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.5 m.s-1, which is -1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);
  }
}

TEST(SpeedLimiterTest, testDecelerationLimits)
{
  {
    // test max_deceleration
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);

    double v = 0.0;
    double limiting_factor = limiter.limit_acceleration(v, 10.0, 0.5);
    // check if the robot speed is now 8.5 m.s-1, which is 10.0 - 3.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 8.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!

    v = 0.0;
    limiting_factor = limiter.limit_acceleration(v, -10.0, 0.5);
    // check if the robot speed is now -8.5 m.s-1, which is -10.0 + 3.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -8.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!
  }
  {
    // test min_deceleration
    // TODO(christophfroehlich): does this behavior make sense?
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    double v = 9.9;
    limiter.limit_acceleration(v, 10.0, 0.5);
    // check if the robot speed is now 9.0m.s-1 = 10 - 2.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 9.0);

    v = -9.9;
    limiter.limit_acceleration(v, -10., 0.5);
    // check if the robot speed is now -9.0m.s-1 = -10 + 2.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -9.0);
  }
  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);
    double v = 0.0;
    double limiting_factor = limiter.limit(v, 10.0, 10.0, 0.5);
    // check if the robot speed is now 8.5 m.s-1, which is 10.0 - 3.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 8.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!

    v = 0.0;
    limiting_factor = limiter.limit(v, -10.0, -10.0, 0.5);
    // check if the robot speed is now -8.5 m.s-1, which is -10.0 + 3.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -8.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!
  }
}

TEST(SpeedLimiterTest, testJerkLimits)
{
  {
    // test max_jerk
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 5.0);
    double v = 10.0;
    double limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 2.5m.s-1 = 5.0m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 2.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 2.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -2.5m.s-1 = -5.0m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, -2.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 2.5 / 10.0);
  }
  {
    // test min_jerk
    // TODO(christophfroehlich): does this behavior make sense?
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 5.0);
    double v = 0.0;
    double limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.25m.s-1 = 0.5m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);  // div by 0!

    v = -std::numeric_limits<double>::epsilon();
    limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.25m.s-1 = -0.5m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
  }
  {
    tricycle_controller::TractionLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0.5, 1.0,
      2.0, 3.0, 0.5, 5.0);
    // acceleration is limiting, not jerk

    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is -1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.5 m.s-1, which is -1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5 / 10.0);
  }
}
