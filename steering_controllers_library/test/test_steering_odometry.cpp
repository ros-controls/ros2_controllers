// Copyright (c) 2023, Virtual Vehicle Research GmbH
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

#define _USE_MATH_DEFINES

#include <gmock/gmock.h>

#include "steering_controllers_library/steering_odometry.hpp"

#include "test_steering_odometry_utils.hpp"

TEST_P(SteeringOdometryTestParameterized, initialize)
{
  if ((test_type_ == TestType::INITIALIZE && odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    // Test constructor
    EXPECT_NO_THROW(steering_odometry::SteeringOdometry());

    // Verify initial state
    EXPECT_DOUBLE_EQ(odom_->get_heading(), 0.);
    EXPECT_DOUBLE_EQ(odom_->get_x(), 0.);
    EXPECT_DOUBLE_EQ(odom_->get_y(), 0.);
  }
  else
  {
    GTEST_SKIP();
  }
}

// ----------------- Ackermann -----------------

TEST_P(SteeringOdometryTestParameterized, AckermannOdometryTest)
{
  if (!(test_type_ == TestType::ODOMETRY && odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  ASSERT_TRUE(ackermann_velocity_.has_value());
  const auto & [v_r, v_l, w_r, w_l, dt] = ackermann_velocity_.value();
  ASSERT_TRUE(odom_->update_from_velocity(v_r, v_l, w_r, w_l, dt));

  // Verify results
  EXPECT_NEAR(odom_->get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom_->get_angular(), 0.1, 1e-3);
  EXPECT_NEAR(odom_->get_x(), 0.1, 1e-3);
  EXPECT_NEAR(odom_->get_heading(), 0.01, 1e-3);
}

TEST_P(SteeringOdometryTestParameterized, ackermann_odometry_openloop_linear)
{
  if (!(test_type_ == TestType::OPEN_LOOP_LINEAR &&
        odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  // Reset to ensure clean state for this test
  odom_->reset_odometry();

  // Use the fixture's odom_ instance
  odom_->update_open_loop(vx_open_, wz_open_, dt_open_);

  EXPECT_DOUBLE_EQ(odom_->get_linear(), vx_open_);
  EXPECT_DOUBLE_EQ(odom_->get_x(), vx_open_ * dt_open_);
  EXPECT_DOUBLE_EQ(odom_->get_y(), 0.0);
}

TEST_P(SteeringOdometryTestParameterized, ackermann_odometry_openloop_angular_left)
{
  if (!(test_type_ == TestType::OPEN_LOOP_LEFT &&
        odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->update_open_loop(vx_open_, wz_open_, dt_open_);

  EXPECT_DOUBLE_EQ(odom_->get_linear(), vx_open_);
  EXPECT_DOUBLE_EQ(odom_->get_angular(), wz_open_);
  EXPECT_GT(odom_->get_x(), 0);  // Should move forward
  EXPECT_GT(odom_->get_y(), 0);  // Should move left
}

TEST_P(SteeringOdometryTestParameterized, ackermann_odometry_openloop_angular_right)
{
  if (!(test_type_ == TestType::OPEN_LOOP_RIGHT &&
        odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->update_open_loop(vx_open_, wz_open_, dt_open_);

  EXPECT_DOUBLE_EQ(odom_->get_linear(), vx_open_);
  EXPECT_DOUBLE_EQ(odom_->get_angular(), wz_open_);
  EXPECT_GT(odom_->get_x(), 0);  // Should move forward
  EXPECT_LT(odom_->get_y(), 0);  // Should move right
}

TEST_P(SteeringOdometryTestParameterized, ackermann_IK_linear)
{
  if (!(test_type_ == TestType::IK_LINEAR && odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  // Set initial state
  odom_->update_open_loop(vx_open_, wz_open_, dt_open_);

  // Get commands
  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_);
  auto cmd0 = std::get<0>(cmd);  // velocity commands
  auto cmd1 = std::get<1>(cmd);  // steering commands

  // Verify results
  EXPECT_EQ(cmd0[0], cmd0[1]);  // Both wheels should have same velocity (linear)
  EXPECT_GT(cmd0[0], 0);        // Positive velocity
  EXPECT_EQ(cmd1[0], cmd1[1]);  // Both wheels should have same steering angle
  EXPECT_EQ(cmd1[0], 0);        // No steering for linear motion
}

TEST_P(SteeringOdometryTestParameterized, ackermann_IK_left)
{
  if (!(test_type_ == TestType::IK_LEFT && odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);

  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_GT(cmd0[0], cmd0[1]);   // right (outer) > left (inner)
  EXPECT_GT(cmd0[0], 0);

  auto cmd1 = std::get<1>(cmd);  // steer
  EXPECT_LT(cmd1[0], cmd1[1]);   // right (outer) < left (inner)
  EXPECT_GT(cmd1[0], 0);
}

TEST_P(SteeringOdometryTestParameterized, ackermann_IK_right)
{
  if (!(test_type_ == TestType::IK_RIGHT && odom_type_ == steering_odometry::ACKERMANN_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);

  auto cmd0 = std::get<0>(cmd);  // vel
  EXPECT_LT(cmd0[0], cmd0[1]);   // right (inner) < left (outer)
  EXPECT_GT(cmd0[0], 0);

  auto cmd1 = std::get<1>(cmd);                     // steer
  EXPECT_GT(std::abs(cmd1[0]), std::abs(cmd1[1]));  // abs right (inner) > abs left (outer)
  EXPECT_LT(cmd1[0], 0);
}

TEST_P(IkSteeringLimitedParameterized, ackermann_IK_right_steering_limited)
{
  // Only run for Ackermann configuration
  if (odom_type_ != steering_odometry::ACKERMANN_CONFIG)
  {
    GTEST_SKIP() << "Skipping non-Ackermann configuration";
  }

  // Test case 1: Already steered
  {
    const auto & [pos, steer, dt] = std::get<0>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<0>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto vel_cmd_steered = std::get<0>(cmd);
    EXPECT_LT(vel_cmd_steered[0], vel_cmd_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_steered[0], 0);

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_GT(std::abs(steer_cmd[0]), std::abs(steer_cmd[1]));  // abs right > abs left
    EXPECT_LT(steer_cmd[0], 0);
  }

  // Test case 2: Not fully steered (baseline)
  std::vector<double> vel_cmd_not_steered;
  {
    const auto & [pos, steer, dt] = std::get<1>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<1>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    vel_cmd_not_steered = std::get<0>(cmd);
    EXPECT_LT(vel_cmd_not_steered[0], vel_cmd_not_steered[1]);
    EXPECT_GT(vel_cmd_not_steered[0], 0);

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_GT(std::abs(steer_cmd[0]), std::abs(steer_cmd[1]));
    EXPECT_LT(steer_cmd[0], 0);
  }

  // Test case 3: Not fully steered with speed reduction
  {
    const auto & [pos, steer, dt] = std::get<2>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<2>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto reduced_vel_cmd = std::get<0>(cmd);
    EXPECT_LT(reduced_vel_cmd[0], reduced_vel_cmd[1]);
    EXPECT_GT(reduced_vel_cmd[0], 0);

    // Verify speed reduction
    for (size_t i = 0; i < reduced_vel_cmd.size(); ++i)
    {
      EXPECT_LT(reduced_vel_cmd[i], vel_cmd_not_steered[i]);
    }

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_GT(std::abs(steer_cmd[0]), std::abs(steer_cmd[1]));
    EXPECT_LT(steer_cmd[0], 0);
  }
}

// ----------------- bicycle -----------------

TEST_P(SteeringOdometryTestParameterized, bicycle_IK_linear)
{
  if (!(test_type_ == TestType::IK_LINEAR && odom_type_ == steering_odometry::BICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_open_loop(ik_vx_, ik_wz_, dt_open_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);

  auto vel_cmd = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(vel_cmd[0], 1.0);  // linear

  auto steer_cmd = std::get<1>(cmd);  // steer
  EXPECT_DOUBLE_EQ(steer_cmd[0], 0);  // linear
}

TEST_P(SteeringOdometryTestParameterized, bicycle_IK_left)
{
  if (!(test_type_ == TestType::IK_LEFT && odom_type_ == steering_odometry::BICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);

  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_GT(cmd1[0], 0);           // right steering
}

TEST_P(SteeringOdometryTestParameterized, bicycle_IK_right)
{
  if (!(test_type_ == TestType::IK_RIGHT && odom_type_ == steering_odometry::BICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);

  auto cmd0 = std::get<0>(cmd);    // vel
  EXPECT_DOUBLE_EQ(cmd0[0], 1.0);  // equals linear
  auto cmd1 = std::get<1>(cmd);    // steer
  EXPECT_LT(cmd1[0], 0);           // left steering
}

TEST_P(IkSteeringLimitedParameterized, bicycle_IK_right_steering_limited)
{
  // Only run for Bicycle configuration
  if (odom_type_ != steering_odometry::BICYCLE_CONFIG)
  {
    GTEST_SKIP() << "Skipping non-Bicycle configuration";
  }

  // Test case 1: Already steered
  {
    const auto & [pos, steer, dt] = std::get<0>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<0>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto vel_cmd_steered = std::get<0>(cmd);
    EXPECT_DOUBLE_EQ(vel_cmd_steered[0], vx);  // equals linear command

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering for right turn
  }

  // Test case 2: Not fully steered (baseline)
  std::vector<double> vel_cmd_not_steered;
  {
    const auto & [pos, steer, dt] = std::get<1>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<1>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    vel_cmd_not_steered = std::get<0>(cmd);
    EXPECT_DOUBLE_EQ(vel_cmd_not_steered[0], vx);  // equals linear command

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering for right turn
  }

  // Test case 3: Not fully steered with speed reduction
  {
    const auto & [pos, steer, dt] = std::get<2>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<2>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto reduced_vel_cmd = std::get<0>(cmd);
    EXPECT_GT(reduced_vel_cmd[0], 0);

    // Verify speed reduction
    for (size_t i = 0; i < reduced_vel_cmd.size(); ++i)
    {
      EXPECT_LT(reduced_vel_cmd[i], vel_cmd_not_steered[i]);
    }

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering for right turn
  }

  // Test case 4: Extreme steering case
  {
    auto pos_case = std::get<3>(position_cases_);
    auto cmd_case = std::get<3>(command_cases_);

    if (pos_case.has_value() && cmd_case.has_value())
    {
      const auto & [pos, steer, dt] = pos_case.value();  // Unpack the tuple
      const auto & [vx, wz, open_loop, reduce_speed] = cmd_case.value();
      odom_->update_from_position(pos, steer, dt);
      auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

      auto extreme_vel_cmd = std::get<0>(cmd);
      EXPECT_GT(extreme_vel_cmd[0], 0);

      auto extreme_steer_cmd = std::get<1>(cmd);
      EXPECT_LT(extreme_steer_cmd[0], 0);
    }
    else
    {
      GTEST_SKIP();
    }
  }
}

TEST_P(SteeringOdometryTestParameterized, bicycle_odometry)
{
  if (!(test_type_ == TestType::ODOMETRY && odom_type_ == steering_odometry::BICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  // Update odometry from velocity
  const auto & [v, w, dt] = *bicycle_velocity_;
  ASSERT_TRUE(odom_->update_from_velocity(v, w, dt));

  // Verify results
  EXPECT_NEAR(odom_->get_linear(), 1.0, 1e-3);
  EXPECT_NEAR(odom_->get_angular(), 0.1, 1e-3);
  EXPECT_NEAR(odom_->get_x(), 0.1, 1e-3);
  EXPECT_NEAR(odom_->get_heading(), 0.01, 1e-3);
}

// ----------------- tricycle -----------------

TEST_P(SteeringOdometryTestParameterized, tricycle_IK_linear)
{
  if (!(test_type_ == TestType::IK_LINEAR && odom_type_ == steering_odometry::TRICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_open_loop(ik_vx_, ik_wz_, dt_open_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);
  auto vel_cmd = std::get<0>(cmd);
  auto steer_cmd = std::get<1>(cmd);

  EXPECT_NEAR(vel_cmd[0], vel_cmd[1], 1e-6);
  EXPECT_GT(vel_cmd[0], 0);
  EXPECT_DOUBLE_EQ(steer_cmd[0], 0);
}

TEST_P(SteeringOdometryTestParameterized, tricycle_IK_left)
{
  if (!(test_type_ == TestType::IK_LEFT && odom_type_ == steering_odometry::TRICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);
  auto vel_cmd = std::get<0>(cmd);
  auto steer_cmd = std::get<1>(cmd);

  EXPECT_GT(vel_cmd[0], vel_cmd[1]);
  EXPECT_GT(vel_cmd[0], 0);
  EXPECT_GT(steer_cmd[0], 0);
}

TEST_P(SteeringOdometryTestParameterized, tricycle_IK_right)
{
  if (!(test_type_ == TestType::IK_RIGHT && odom_type_ == steering_odometry::TRICYCLE_CONFIG))
  {
    GTEST_SKIP();
  }

  odom_->reset_odometry();
  odom_->update_from_position(pos_, steer_pos_, dt_pos_);

  auto cmd = odom_->get_commands(ik_vx_, ik_wz_, open_loop_, reduce_speed_);
  auto vel_cmd = std::get<0>(cmd);
  auto steer_cmd = std::get<1>(cmd);

  EXPECT_LT(vel_cmd[0], vel_cmd[1]);
  EXPECT_GT(vel_cmd[0], 0);
  EXPECT_LT(steer_cmd[0], 0);
}

TEST_P(IkSteeringLimitedParameterized, tricycle_IK_right_steering_limited)
{
  // Only run for Tricycle configuration
  if (odom_type_ != steering_odometry::TRICYCLE_CONFIG)
  {
    GTEST_SKIP() << "Skipping non-Tricycle configuration";
  }

  // Test case 1: Already steered (-0.785 rad)
  {
    const auto & [pos, steer, dt] = std::get<0>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<0>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto vel_cmd_steered = std::get<0>(cmd);
    EXPECT_LT(vel_cmd_steered[0], vel_cmd_steered[1]);  // right (inner) < left (outer)
    EXPECT_GT(vel_cmd_steered[0], 0);

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering for right turn
  }

  // Test case 2: Not fully steered (-0.1 rad, baseline)
  std::vector<double> vel_cmd_not_steered;
  {
    const auto & [pos, steer, dt] = std::get<1>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<1>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    vel_cmd_not_steered = std::get<0>(cmd);
    EXPECT_LT(vel_cmd_not_steered[0], vel_cmd_not_steered[1]);  // right < left
    EXPECT_GT(vel_cmd_not_steered[0], 0);

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering
  }

  // Test case 3: Not fully steered with speed reduction (-0.1 rad)
  {
    const auto & [pos, steer, dt] = std::get<2>(position_cases_);
    const auto & [vx, wz, open_loop, reduce_speed] = std::get<2>(command_cases_);

    odom_->update_from_position(pos, steer, dt);
    auto cmd = odom_->get_commands(vx, wz, open_loop, reduce_speed);

    auto reduced_vel_cmd = std::get<0>(cmd);
    EXPECT_LT(reduced_vel_cmd[0], reduced_vel_cmd[1]);  // right < left
    EXPECT_GT(reduced_vel_cmd[0], 0);

    // Verify speed reduction compared to baseline
    for (size_t i = 0; i < reduced_vel_cmd.size(); ++i)
    {
      EXPECT_LT(reduced_vel_cmd[i], vel_cmd_not_steered[i]);
    }

    auto steer_cmd = std::get<1>(cmd);
    EXPECT_LT(steer_cmd[0], 0);  // Negative steering
  }
}
TEST_P(SteeringOdometryTestParameterized, tricycle_odometry)
{
  if (!(odom_type_ == steering_odometry::TRICYCLE_CONFIG && test_type_ == TestType::ODOMETRY))
  {
    GTEST_SKIP();
  }

  const auto & [v_l, v_r, w, dt] = *tricycle_velocity_;
  ASSERT_TRUE(odom_->update_from_velocity(v_l, v_r, w, dt));

  EXPECT_NEAR(odom_->get_linear(), 1.002, 1e-3);
  EXPECT_NEAR(odom_->get_angular(), .1, 1e-3);
  EXPECT_NEAR(odom_->get_x(), .1, 1e-3);
  EXPECT_NEAR(odom_->get_heading(), .01, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
  SteeringOdometryParameterizedTests, SteeringOdometryTestParameterized,
  ::testing::Values(
    // ==================== INITIALIZATION TESTS ====================
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,      // odom_type
      std::make_tuple(1., 2., 3.),              // wheel_radius, wheelbase, track_width
      std::make_tuple(0.0, 0.0, 0.0),           // position (pos, steer, dt)
      std::nullopt,                             // ackermann velocity (v_r, v_l, w_r, w_l, dt)
      std::nullopt,                             // bicycle_velocity (none for init)
      std::nullopt,                             // tricycle_velocity (none for init)
      std::make_tuple(0.0, 0.0, 0.0),           // open_loop (vx, ωz, dt)
      std::make_tuple(0.0, 0.0, false, false),  // ik_params
      TestType::INITIALIZE),

    // ==================== ACKERMANN CONFIG TESTS ====================
    // Ackermann Odometry
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG, std::make_tuple(1., 1., 1.),
      std::make_tuple(0., 0., 0.),            // initial position
      std::make_tuple(1., 1., .1, .1, .1),    // ackermann velocity (v_r, v_l, w_r, w_l, dt)
      std::nullopt,                           // bicycle_velocity
      std::nullopt,                           // tricycle_velocity
      std::make_tuple(0., 0., 0.),            // open_loop (unused)
      std::make_tuple(0., 0., false, false),  // ik_params (unused)
      TestType::ODOMETRY),

    // Ackermann Open-Loop Linear
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG, std::make_tuple(1., 2., 1.),
      std::make_tuple(0., 0., 0.),           // initial position
      std::nullopt,                          // ackermann_velocity
      std::nullopt,                          // bicycle_velocity
      std::nullopt,                          // tricycle_velocity
      std::make_tuple(2., 0., 0.5),          // open_loop (vx, ωz, dt)
      std::make_tuple(0., 0., true, false),  // ik_params (unused)
      TestType::OPEN_LOOP_LINEAR),

    // Ackermann Open-Loop Left
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,    // odom_type
      std::make_tuple(1., 2., 1.),            // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),            // position (unused)
      std::nullopt,                           // ackermann_velocity (unused)
      std::nullopt,                           // bicycle_velocity (unused)
      std::nullopt,                           // tricycle_velocity (unused)
      std::make_tuple(1., 1., 1.),            // open_loop (vx, ωz, dt)
      std::make_tuple(0., 0., false, false),  // ik_params (unused)
      TestType::OPEN_LOOP_LEFT                // test type
      ),

    // Ackermann Open-Loop Right
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,    // odom_type
      std::make_tuple(1., 2., 1.),            // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),            // position (unused)
      std::nullopt,                           // ackermann_velocity (unused)
      std::nullopt,                           // bicycle_velocity (unused)
      std::nullopt,                           // tricycle_velocity (unused)
      std::make_tuple(1., -1., 1.),           // open_loop (vx, ωz, dt)
      std::make_tuple(0., 0., false, false),  // ik_params (unused)
      TestType::OPEN_LOOP_RIGHT               // test type
      ),

    // Ackermann IK_Linear
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,   // odom_type
      std::make_tuple(1., 2., 1.),           // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),           // position (unused)
      std::nullopt,                          // ackermann_velocity (unused)
      std::nullopt,                          // bicycle_velocity (unused)
      std::nullopt,                          // tricycle_velocity (unused)
      std::make_tuple(1., 0., 1.),           // open_loop (vx, ωz, dt)
      std::make_tuple(1., 0., true, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LINEAR                    // test type
      ),

    // Ackermann IK_Left
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,     // odom_type
      std::make_tuple(1., 2., 1.),             // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                            // ackermann_velocity (unused)
      std::nullopt,                            // bicycle_velocity (unused)
      std::nullopt,                            // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),             // open_loop (unused)
      std::make_tuple(1., 0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LEFT                        // test type
      ),

    // Ackermann IK_right
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG,      // odom_type
      std::make_tuple(1., 2., 1.),              // wheel_radius, wheelbase, track_width
      std::make_tuple(0., -0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                             // ackermann_velocity (unused)
      std::nullopt,                             // bicycle_velocity (unused)
      std::nullopt,                             // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),              // open_loop (unused)
      std::make_tuple(1., -0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_RIGHT                        // test type
      ),

    // ==================== BICYCLE CONFIG TESTS ====================
    // Bicycle Odometry
    std::make_tuple(
      steering_odometry::BICYCLE_CONFIG,      // odom_type
      std::make_tuple(1., 1., 1.),            // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),            // position (unused)
      std::nullopt,                           // ackermann_velocity (unused)
      std::make_tuple(1., 0.1, 0.1),          // bicycle_velocity (v, ω, dt)
      std::nullopt,                           // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),            // open_loop (unused)
      std::make_tuple(0., 0., false, false),  // ik_params (unused)
      TestType::ODOMETRY                      // test type
      ),

    // Bicycle IK_linear
    std::make_tuple(
      steering_odometry::BICYCLE_CONFIG,     // odom_type
      std::make_tuple(1., 2., 1.),           // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),           // position (unused)
      std::nullopt,                          // ackermann_velocity (unused)
      std::nullopt,                          // bicycle_velocity (unused)
      std::nullopt,                          // tricycle_velocity (unused)
      std::make_tuple(1., 0., 1.),           // open_loop (vx, ωz, dt)
      std::make_tuple(1., 0., true, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LINEAR                    // test type
      ),

    // Bicycle IK_left
    std::make_tuple(
      steering_odometry::BICYCLE_CONFIG,       // odom_type
      std::make_tuple(1., 2., 1.),             // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                            // ackermann_velocity (unused)
      std::nullopt,                            // bicycle_velocity (unused)
      std::nullopt,                            // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),             // open_loop (unused)
      std::make_tuple(1., 0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LEFT                        // test type
      ),

    // Bicycle IK_right
    std::make_tuple(
      steering_odometry::BICYCLE_CONFIG,        // odom_type
      std::make_tuple(1., 2., 1.),              // wheel_radius, wheelbase, track_width
      std::make_tuple(0., -0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                             // ackermann_velocity (unused)
      std::nullopt,                             // bicycle_velocity (unused)
      std::nullopt,                             // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),              // open_loop (unused)
      std::make_tuple(1., -0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_RIGHT                        // test type
      ),

    // ==================== TRICYCLE CONFIG TESTS ====================
    // Tricycle Odometry
    std::make_tuple(
      steering_odometry::TRICYCLE_CONFIG,     // odom_type
      std::make_tuple(1., 1., 1.),            // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),            // position (unused)
      std::nullopt,                           // ackermann_velocity (unused)
      std::nullopt,                           // bicycle_velocity (unused)
      std::make_tuple(1., 1., .1, .1),        // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),            // open_loop (unused)
      std::make_tuple(0., 0., false, false),  // ik_params (unused)
      TestType::ODOMETRY                      // test type
      ),

    // Tricycle IK Linear
    std::make_tuple(
      steering_odometry::TRICYCLE_CONFIG,    // odom_type
      std::make_tuple(1., 2., 1.),           // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0., 0.),           // position (unused)
      std::nullopt,                          // ackermann_velocity (unused)
      std::nullopt,                          // bicycle_velocity (unused)
      std::nullopt,                          // tricycle_velocity (unused)
      std::make_tuple(1., 0., 1.),           // open_loop (vx, ωz, dt)
      std::make_tuple(1., 0., true, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LINEAR                    // test type
      ),

    // Tricycle IK_left
    std::make_tuple(
      steering_odometry::TRICYCLE_CONFIG,      // odom_type
      std::make_tuple(1., 2., 1.),             // wheel_radius, wheelbase, track_width
      std::make_tuple(0., 0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                            // ackermann_velocity (unused)
      std::nullopt,                            // bicycle_velocity (unused)
      std::nullopt,                            // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),             // open_loop (unused)
      std::make_tuple(1., 0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_LEFT                        // test type
      ),

    // Tricycle IK_right
    std::make_tuple(
      steering_odometry::TRICYCLE_CONFIG,       // odom_type
      std::make_tuple(1., 2., 1.),              // wheel_radius, wheelbase, track_width
      std::make_tuple(0., -0.2, 1.),            // position (x, steer, dt)
      std::nullopt,                             // ackermann_velocity (unused)
      std::nullopt,                             // bicycle_velocity (unused)
      std::nullopt,                             // tricycle_velocity (unused)
      std::make_tuple(0., 0., 0.),              // open_loop (unused)
      std::make_tuple(1., -0.1, false, false),  // ik_params (vx, ωz, open_loop, reduce_speed)
      TestType::IK_RIGHT                        // test type
      )

      ),
  [](const ::testing::TestParamInfo<SteeringOdometryTestParameterized::ParamType> & test_info)
  {
    std::string name;
    const auto & params = test_info.param;

    // Configuration type
    switch (static_cast<int>(std::get<0>(params)))
    {
      case steering_odometry::ACKERMANN_CONFIG:
        name = "Ackermann";
        break;
      case steering_odometry::BICYCLE_CONFIG:
        name = "Bicycle";
        break;
      case steering_odometry::TRICYCLE_CONFIG:
        name = "Tricycle";
        break;
      default:
        name = "Unknown";
        break;
    }

    // Add unique identifier (e.g., test index)
    name += "_" + std::to_string(test_info.index);  // ← This ensures uniqueness
    // Test type
    switch (std::get<8>(params))
    {
      case TestType::INITIALIZE:
        name += "_Init";
        break;
      case TestType::ODOMETRY:
        name += "_Odom";
        break;
      case TestType::OPEN_LOOP_LINEAR:
        name += "_OpenLinear";
        break;
      case TestType::OPEN_LOOP_LEFT:
        name += "_OpenLeft";
        break;
      case TestType::OPEN_LOOP_RIGHT:
        name += "_OpenRight";
        break;
      case TestType::IK_LINEAR:
        name += "_IKLinear";
        break;
      case TestType::IK_LEFT:
        name += "_IKLeft";
        break;
      case TestType::IK_RIGHT:
        name += "_IKRight";
        break;
    }

    return name;
  });

INSTANTIATE_TEST_SUITE_P(
  IkSteeringLimitedParameterizedTests, IkSteeringLimitedParameterized,
  ::testing::Values(
    // Ackermann configuration (3 cases)
    std::make_tuple(
      steering_odometry::ACKERMANN_CONFIG, std::make_tuple(1., 2., 1.),
      std::make_tuple(                    // Position cases
        std::make_tuple(0., -0.785, 1.),  // Case 1
        std::make_tuple(0., -0.1, 1.),    // Case 2
        std::make_tuple(0., -0.1, 1.),    // Case 3
        std::nullopt                      // No Case 4
        ),
      std::make_tuple(  // Command cases
        std::make_tuple(1., -0.5, false, true), std::make_tuple(1., -0.5, false, false),
        std::make_tuple(1., -0.5, false, true),
        std::nullopt  // No Case 4
        )),

    // Bicycle configuration (4 cases)
    std::make_tuple(
      steering_odometry::BICYCLE_CONFIG, std::make_tuple(1., 2., 1.),
      std::make_tuple(
        std::make_tuple(0., -0.785, 1.),                     // Case 1
        std::make_tuple(0., -0.1, 1.),                       // Case 2
        std::make_tuple(0., -0.1, 1.),                       // Case 3
        std::make_optional(std::make_tuple(0.0, M_PI, 1.0))  // Case 4
        ),
      std::make_tuple(
        std::make_tuple(1., -0.5, false, true), std::make_tuple(1., -0.5, false, false),
        std::make_tuple(1., -0.5, false, true),
        std::make_optional(std::make_tuple(1., -0.5, false, true))  // Case 4
        )),

    // Tricycle configuration
    std::make_tuple(
      steering_odometry::TRICYCLE_CONFIG, std::make_tuple(1., 2., 1.0),
      std::make_tuple(
        std::make_tuple(0., -0.785, 1.),  // Case 1
        std::make_tuple(0., -0.1, 1.),    // Case 2
        std::make_tuple(0., -0.1, 1.),    // Case 3
        std::nullopt                      // No Case 4
        ),
      std::make_tuple(
        std::make_tuple(1., -0.5, false, true), std::make_tuple(1., -0.5, false, false),
        std::make_tuple(1., -0.5, false, true),
        std::nullopt  // No Case 4
        ))),

  [](const auto & test_info)
  {
    std::string name;
    const auto & params = test_info.param;

    // Add configuration type
    switch (std::get<0>(params))
    {
      case steering_odometry::ACKERMANN_CONFIG:
        name = "ackermann";
        break;
      case steering_odometry::BICYCLE_CONFIG:
        name = "bicycle";
        break;
      case steering_odometry::TRICYCLE_CONFIG:
        name = "tricycle";
        break;
      default:
        name = "unknownConfig";
        break;
    }

    // Add test identifier
    name += "_IK_right_steering_limited";
    return name;
  });
