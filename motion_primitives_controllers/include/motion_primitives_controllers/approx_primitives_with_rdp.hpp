// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#ifndef MOTION_PRIMITIVES_CONTROLLERS__APPROX_PRIMITIVES_WITH_RDP_HPP_
#define MOTION_PRIMITIVES_CONTROLLERS__APPROX_PRIMITIVES_WITH_RDP_HPP_

#include <string>
#include <vector>
#include "control_msgs/msg/motion_argument.hpp"
#include "control_msgs/msg/motion_primitive.hpp"
#include "control_msgs/msg/motion_primitive_sequence.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "motion_primitives_controllers/rdp.hpp"

using MotionSequence = control_msgs::msg::MotionPrimitiveSequence;

namespace approx_primitives_with_rdp
{

struct PlannedTrajectoryPoint
{
  double time_from_start;
  std::vector<double> joint_positions;
  geometry_msgs::msg::Pose pose;
};

// Approximate with LIN Primitives in Cartesian Space
MotionSequence approxLinPrimitivesWithRDP(
  const std::vector<PlannedTrajectoryPoint> & trajectory, double epsilon_position,
  double epsilon_angle, double cart_vel, double cart_acc, bool use_time_not_vel_and_acc = false,
  double blend_overwrite = -1.0, double blend_scale = 0.1, double blend_lower_limit = 0.0,
  double blend_upper_limit = 1.0);

// Approximate with PTP Primitives in Joint Space
MotionSequence approxPtpPrimitivesWithRDP(
  const std::vector<PlannedTrajectoryPoint> & trajectory, double epsilon, double joint_vel,
  double joint_acc, bool use_time_not_vel_and_acc = false, double blend_overwrite = -1.0,
  double blend_scale = 0.1, double blend_lower_limit = 0.0, double blend_upper_limit = 1.0);

double calculateBlendRadius(
  const rdp::Point & previous_point, const rdp::Point & current_point,
  const rdp::Point & next_point, double blend_scale, double blend_lower_limit,
  double blend_upper_limit);
}  // namespace approx_primitives_with_rdp

#endif  // MOTION_PRIMITIVES_CONTROLLERS__APPROX_PRIMITIVES_WITH_RDP_HPP_
