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

#include "motion_primitives_forward_controller/approx_primitives_with_rdp.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

using control_msgs::msg::MotionArgument;
using control_msgs::msg::MotionPrimitive;
using control_msgs::msg::MotionPrimitiveSequence;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

namespace approx_primitives_with_rdp
{

MotionSequence approxLinPrimitivesWithRDP(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  double epsilon_position, double epsilon_angle, double cart_vel, double cart_acc,
  bool use_time_not_vel_and_acc, double blend_overwrite)
{
  MotionSequence motion_sequence;
  std::vector<MotionPrimitive> motion_primitives;

  if (trajectory.empty())
  {
    RCLCPP_WARN(
      rclcpp::get_logger("approx_primitives_with_rdp"),
      "[approxLinPrimitivesWithRDP] Warning: trajectory is empty.");
    return motion_sequence;
  }

  // Reduce positions using RDP
  rdp::PointList cartesian_points;
  for (const auto & point : trajectory)
  {
    cartesian_points.push_back(
      {point.pose.position.x, point.pose.position.y, point.pose.position.z});
  }

  auto [reduced_points, reduced_indices] = rdp::rdpRecursive(cartesian_points, epsilon_position);

  RCLCPP_INFO(
    rclcpp::get_logger("approx_primitives_with_rdp"),
    "[approxLinPrimitivesWithRDP] Added %zu points due to position change (epsilon_position = "
    "%.4f).",
    reduced_indices.size(), epsilon_position);

  std::set<size_t> final_indices(reduced_indices.begin(), reduced_indices.end());
  std::set<size_t> position_indices(reduced_indices.begin(), reduced_indices.end());
  std::set<size_t> orientation_indices;

  // Enrich reduced_indices by checking quaternion changes in each segment
  for (size_t i = 1; i < reduced_indices.size(); ++i)
  {
    size_t start_index = reduced_indices[i - 1];
    size_t end_index = reduced_indices[i];

    if (end_index - start_index <= 1) continue;  // nothing in between

    // Extract quaternion segment for RDP
    std::vector<geometry_msgs::msg::Quaternion> quats;
    for (size_t j = start_index; j <= end_index; ++j)
    {
      quats.push_back(trajectory[j].pose.orientation);
    }

    auto [reduced_quats, reduced_quat_indices] =
      rdp::rdpRecursiveQuaternion(quats, epsilon_angle, start_index);

    // Add new orientation indices to final set
    for (size_t idx : reduced_quat_indices)
    {
      if (final_indices.insert(idx).second)  // true if inserted (i.e., newly added)
      {
        orientation_indices.insert(idx);
      }
    }
  }

  // Sort final indices for ordered primitive generation
  std::vector<size_t> sorted_final_indices(final_indices.begin(), final_indices.end());
  std::sort(sorted_final_indices.begin(), sorted_final_indices.end());

  RCLCPP_INFO(
    rclcpp::get_logger("approx_primitives_with_rdp"),
    "[approxLinPrimitivesWithRDP] Added %zu additional intermediate points due to orientation "
    "angle change (epsilon_angle = %.4f).",
    sorted_final_indices.size() - reduced_indices.size(), epsilon_angle);

  // Generate motion primitives between reduced points
  for (size_t i = 1; i < sorted_final_indices.size(); ++i)
  {
    size_t start_index = sorted_final_indices[i - 1];
    size_t end_index = sorted_final_indices[i];

    MotionPrimitive primitive;
    primitive.type = MotionPrimitive::LINEAR_CARTESIAN;

    // Calculate blend radius based on the distance to the next and previous point
    if (i == sorted_final_indices.size() - 1)
    {
      primitive.blend_radius = 0.0;  // Last point has no blend radius
    }
    else
    {
      if (blend_overwrite > 0.0)
      {
        primitive.blend_radius = blend_overwrite;
      }
      else
      {
        const auto & p0 = trajectory[start_index].pose.position;
        const auto & p1 = trajectory[end_index].pose.position;
        const auto & p2 = trajectory[sorted_final_indices[i + 1]].pose.position;
        primitive.blend_radius =
          calculateBlendRadius({p0.x, p0.y, p0.z}, {p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z});
      }
    }

    double velocity = -1.0;
    double acceleration = -1.0;
    double move_time = -1.0;

    // Use time or velocity+acceleration based on use_time_not_vel_and_acc
    if (use_time_not_vel_and_acc)
    {
      move_time = trajectory[end_index].time_from_start - trajectory[start_index].time_from_start;
      MotionArgument arg_time;
      arg_time.name = "move_time";
      arg_time.value = move_time;
      primitive.additional_arguments.push_back(arg_time);
    }
    else
    {
      velocity = cart_vel;
      acceleration = cart_acc;
      MotionArgument arg_vel;
      arg_vel.name = "velocity";
      arg_vel.value = velocity;
      primitive.additional_arguments.push_back(arg_vel);

      MotionArgument arg_acc;
      arg_acc.name = "acceleration";
      arg_acc.value = acceleration;
      primitive.additional_arguments.push_back(arg_acc);
    }

    // Add pose to primitive
    PoseStamped pose_stamped;
    pose_stamped.pose = trajectory[end_index].pose;
    primitive.poses.push_back(pose_stamped);
    motion_primitives.push_back(primitive);

    // Determine reason for addition
    std::string reason;
    bool pos = position_indices.count(end_index);
    bool ori = orientation_indices.count(end_index);
    if (pos && ori)
      reason = "position+orientation";
    else if (pos)
      reason = "position";
    else if (ori)
      reason = "orientation";
    else
      reason = "unknown";

    RCLCPP_DEBUG(
      rclcpp::get_logger("approx_primitives_with_rdp"),
      "Added LIN Primitive [%zu] (%s): (x,y,z,qx,qy,qz,qw) = (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, "
      "%.4f), blend_radius = %.4f, move_time = %.4f, velocity = %.4f, acceleration = %.4f",
      i, reason.c_str(), pose_stamped.pose.position.x, pose_stamped.pose.position.y,
      pose_stamped.pose.position.z, pose_stamped.pose.orientation.x,
      pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z,
      pose_stamped.pose.orientation.w, primitive.blend_radius, move_time, velocity, acceleration);
  }

  motion_sequence.motions = motion_primitives;

  return motion_sequence;
}

MotionSequence approxPtpPrimitivesWithRDP(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  double epsilon, double joint_vel, double joint_acc, bool use_time_not_vel_and_acc,
  double blend_overwrite)
{
  MotionSequence motion_sequence;
  std::vector<MotionPrimitive> motion_primitives;

  if (trajectory.empty())
  {
    RCLCPP_WARN(
      rclcpp::get_logger("approx_primitives_with_rdp"),
      "[approxPtpPrimitivesWithRDP] Warning: trajectory is empty.");
    return motion_sequence;
  }

  // Collect joint positions for RDP
  rdp::PointList points;
  for (const auto & pt : trajectory)
  {
    points.push_back(pt.joint_positions);
  }

  auto [reduced_points, reduced_indices] = rdp::rdpRecursive(points, epsilon);

  // Generate motion primitives between reduced joint points
  for (size_t i = 1; i < reduced_points.size(); ++i)
  {
    MotionPrimitive primitive;
    primitive.type = MotionPrimitive::LINEAR_JOINT;

    // Calculate blend radius based on the distance to the next and previous point
    if (i == reduced_points.size() - 1)
    {
      primitive.blend_radius = 0.0;  // Last point has no blend radius
    }
    else
    {
      if (blend_overwrite > 0.0)
      {
        primitive.blend_radius = blend_overwrite;
      }
      else
      {
        size_t prev_index = reduced_indices[i - 1];
        size_t curr_index = reduced_indices[i];
        size_t next_index = reduced_indices[i + 1];

        rdp::Point prev_xyz = {
          trajectory[prev_index].pose.position.x, trajectory[prev_index].pose.position.y,
          trajectory[prev_index].pose.position.z};
        rdp::Point curr_xyz = {
          trajectory[curr_index].pose.position.x, trajectory[curr_index].pose.position.y,
          trajectory[curr_index].pose.position.z};
        rdp::Point next_xyz = {
          trajectory[next_index].pose.position.x, trajectory[next_index].pose.position.y,
          trajectory[next_index].pose.position.z};
        primitive.blend_radius = calculateBlendRadius(prev_xyz, curr_xyz, next_xyz);
      }
    }

    double velocity = -1.0;
    double acceleration = -1.0;
    double move_time = -1.0;

    // Use time or velocity+acceleration based on use_time_not_vel_and_acc
    if (use_time_not_vel_and_acc)
    {
      size_t start_index = reduced_indices[i - 1];
      size_t end_index = reduced_indices[i];
      double prev_time = trajectory[start_index].time_from_start;
      double curr_time = trajectory[end_index].time_from_start;
      move_time = curr_time - prev_time;

      MotionArgument arg_time;
      arg_time.name = "move_time";
      arg_time.value = move_time;
      primitive.additional_arguments.push_back(arg_time);
    }
    else
    {
      velocity = joint_vel;
      acceleration = joint_acc;

      MotionArgument arg_vel;
      arg_vel.name = "velocity";
      arg_vel.value = velocity;
      primitive.additional_arguments.push_back(arg_vel);

      MotionArgument arg_acc;
      arg_acc.name = "acceleration";
      arg_acc.value = acceleration;
      primitive.additional_arguments.push_back(arg_acc);
    }

    primitive.joint_positions = reduced_points[i];
    motion_primitives.push_back(primitive);

    std::ostringstream joint_stream;
    joint_stream << "Added PTP Primitive [" << i << "]: joints = (";
    for (size_t j = 0; j < reduced_points[i].size(); ++j)
    {
      joint_stream << reduced_points[i][j];
      if (j + 1 < reduced_points[i].size()) joint_stream << ", ";
    }
    joint_stream << "), blend_radius = " << primitive.blend_radius << ", move_time = " << move_time
                 << ", velocity = " << velocity << ", acceleration = " << acceleration;

    RCLCPP_DEBUG(
      rclcpp::get_logger("approx_primitives_with_rdp"), "%s", joint_stream.str().c_str());
  }

  motion_sequence.motions = motion_primitives;

  RCLCPP_INFO(
    rclcpp::get_logger("approx_primitives_with_rdp"),
    "Reduced %zu joint points to %zu PTP primitives with epsilon=%.4f", points.size(),
    reduced_points.size() - 1, epsilon);

  return motion_sequence;
}

double calculateBlendRadius(
  const rdp::Point & previous_point, const rdp::Point & current_point,
  const rdp::Point & next_point)
{
  double dist_prev = std::sqrt(
    std::pow(current_point[0] - previous_point[0], 2) +
    std::pow(current_point[1] - previous_point[1], 2) +
    std::pow(current_point[2] - previous_point[2], 2));

  double dist_next = std::sqrt(
    std::pow(next_point[0] - current_point[0], 2) + std::pow(next_point[1] - current_point[1], 2) +
    std::pow(next_point[2] - current_point[2], 2));

  double min_dist = std::min(dist_prev, dist_next);
  double blend = 0.1 * min_dist;

  // Clamp blend radius to [0.01, 0.1]
  if (blend < 0.01)
  {
    blend = 0.01;
  }
  else if (blend > 0.1)
  {
    blend = 0.1;
  }

  return blend;
}

}  // namespace approx_primitives_with_rdp
