// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_STEERING_ODOMETRY_UTILS_HPP
#define TEST_STEERING_ODOMETRY_UTILS_HPP

#include <gtest/gtest.h>
#include <memory>
#include <tuple>
#include <variant>
#include <utility>
#include "steering_controllers_library/steering_odometry.hpp"

enum class TestType {
  INITIALIZE,
  ODOMETRY,
  OPEN_LOOP_LINEAR,
  OPEN_LOOP_LEFT,
  OPEN_LOOP_RIGHT,
  IK_LINEAR,
  IK_LEFT,
  IK_RIGHT
};

class SteeringOdometryTestParameterized
    : public ::testing::TestWithParam<std::tuple<
          unsigned int,  // odom_type (ACKERMANN_CONFIG, BICYCLE_CONFIG, TRICYCLE_CONFIG)
          std::tuple<double, double, double>,  // wheel_radius, wheelbase, track_width
          std::tuple<double, double, double>,  // position (pos, steer, dt)
          std::optional<std::tuple<double, double, double, double, double>>,  // ackermann velocity (v_r, v_l, w_r, w_l, dt)
          std::optional<std::tuple<double,double,double>>, // bicycle odometry (v,w,dt)
          std::optional<std::tuple<double, double, double, double>>,  // tricycle velocity (v_r, v_l, steer, dt)
          std::tuple<double, double, double>,  // open_loop (vx, ωz, dt)
          std::tuple<double, double, bool, bool>,  // ik_params (vx, ωz, open_loop, reduce_speed)
          TestType>> {
protected:
    void SetUp() override {
        const auto& params = GetParam();
        odom_type_ = std::get<0>(params);
        std::tie(wheel_radius_, wheelbase_, track_width_) = std::get<1>(params);
        std::tie(pos_, steer_pos_, dt_pos_) = std::get<2>(params);
        ackermann_velocity_ = std::get<3>(params);
        bicycle_velocity_ = std::get<4>(params);
        tricycle_velocity_ = std::get<5>(params);

        std::tie(vx_open_, wz_open_, dt_open_) = std::get<6>(params);
        std::tie(ik_vx_, ik_wz_, open_loop_, reduce_speed_) = std::get<7>(params);
        test_type_ = std::get<8>(params);

        odom_ = std::make_unique<steering_odometry::SteeringOdometry>(1);
        odom_->set_wheel_params(wheel_radius_, wheelbase_, track_width_);
        odom_->set_odometry_type(odom_type_);
    }

protected:
    // Parameters
    unsigned int odom_type_;
    double wheel_radius_, wheelbase_, track_width_;
    double pos_, steer_pos_, dt_pos_;
    std::optional<std::tuple<double,double,double,double,double>> ackermann_velocity_;
    std::optional<std::tuple<double, double, double>> bicycle_velocity_;
    std::optional<std::tuple<double, double, double, double>> tricycle_velocity_;
    double vx_open_, wz_open_, dt_open_;
    double ik_vx_, ik_wz_;
    bool open_loop_, reduce_speed_;
    TestType test_type_;

    std::unique_ptr<steering_odometry::SteeringOdometry> odom_;
};

class IkSteeringLimitedParameterized
  : public ::testing::Test,
    public ::testing::WithParamInterface<std::tuple<
      unsigned int,  // Odometry type
      std::tuple<double, double, double>,  // wheel_radius, wheelbase, track_width
      std::tuple<  // Position cases (nested tuple)
        std::tuple<double, double, double>,  // Case 1
        std::tuple<double, double, double>,  // Case 2
        std::tuple<double, double, double>,  // Case 3
        std::optional<std::tuple<double, double, double>>  // Optional Case 4
      >,
      std::tuple<  // Command cases (nested tuple)
        std::tuple<double, double, bool, bool>,  // Case 1
        std::tuple<double, double, bool, bool>,  // Case 2
        std::tuple<double, double, bool, bool>,  // Case 3
        std::optional<std::tuple<double, double, bool, bool>>  // Optional Case 4
      >
    >>
{
protected:
  void SetUp() override {
    const auto& params = GetParam();
    odom_type_ = std::get<0>(params);
    std::tie(wheel_radius_, wheelbase_, track_width_) = std::get<1>(params);
    position_cases_ = std::get<2>(params);
    command_cases_ = std::get<3>(params);

    odom_ = std::make_unique<steering_odometry::SteeringOdometry>(1);
    odom_->reset_odometry();
    odom_->set_wheel_params(wheel_radius_, wheelbase_, track_width_);
    odom_->set_odometry_type(odom_type_);
  }

protected:
  double wheel_radius_;
  double wheelbase_;
  double track_width_;
  unsigned int odom_type_;
  std::tuple<
    std::tuple<double, double, double>,  // Case 1
    std::tuple<double, double, double>,  // Case 2
    std::tuple<double, double, double>,  // Case 3
    std::optional<std::tuple<double, double, double>>  // Optional Case 4
  > position_cases_;
  std::tuple<
    std::tuple<double, double, bool, bool>,  // Case 1
    std::tuple<double, double, bool, bool>,  // Case 2
    std::tuple<double, double, bool, bool>,  // Case 3
    std::optional<std::tuple<double, double, bool, bool>>  // Optional Case 4
  > command_cases_;
  std::unique_ptr<steering_odometry::SteeringOdometry> odom_;
};

#endif // TEST_STEERING_ODOMETRY_UTILS_HPP
