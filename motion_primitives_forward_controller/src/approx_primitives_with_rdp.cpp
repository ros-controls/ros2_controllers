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

using control_msgs::msg::MotionArgument;
using control_msgs::msg::MotionPrimitive;
using control_msgs::msg::MotionPrimitiveSequence;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

namespace approx_primitives_with_rdp
{

MotionSequence approxLinPrimitivesWithRDP(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  double epsilon_position, double epsilon_angle, bool use_time_not_vel_and_acc)
{
  MotionSequence motion_sequence;
  std::vector<MotionPrimitive> motion_primitives;

  if (trajectory.empty())
  {
    std::cerr << "[approxLinPrimitivesWithRDP] Warning: trajectory is empty." << std::endl;
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

  std::cout << "[approxLinPrimitivesWithRDP] Added " << reduced_indices.size()
            << " points due to position change (epsilon_position = " << epsilon_position << ")."
            << std::endl;

  std::set<size_t> final_indices(reduced_indices.begin(), reduced_indices.end());
  std::set<size_t> position_indices(reduced_indices.begin(), reduced_indices.end());
  std::set<size_t> orientation_indices;

  // Enrich reduced_indices by checking quaternion changes in each segment
  for (size_t i = 1; i < reduced_indices.size(); ++i)
  {
    size_t start_index = reduced_indices[i - 1];
    size_t end_index = reduced_indices[i];

    if (end_index - start_index <= 1) continue;  // nothing in between

    // Extract quaternion segment
    std::vector<geometry_msgs::msg::Quaternion> quats;
    for (size_t j = start_index; j <= end_index; ++j)
    {
      quats.push_back(trajectory[j].pose.orientation);
    }

    auto [reduced_quats, reduced_quat_indices] =
      rdp::rdpRecursiveQuaternion(quats, epsilon_angle, start_index);

    for (size_t idx : reduced_quat_indices)
    {
      if (final_indices.insert(idx).second)  // true if inserted (i.e., newly added)
      {
        orientation_indices.insert(idx);
      }
    }
  }

  std::vector<size_t> sorted_final_indices(final_indices.begin(), final_indices.end());
  std::sort(sorted_final_indices.begin(), sorted_final_indices.end());

  std::cout << "[approxLinPrimitivesWithRDP] Added "
            << sorted_final_indices.size() - reduced_indices.size()
            << " additional intermediate points due to orientation angle change (epsilon_angle = "
            << epsilon_angle << ")." << std::endl;

  // Compute cartesian velocity and acceleration between trajectory points
  std::vector<double> velocities, accelerations;
  calculateCartVelAndAcc(trajectory, velocities, accelerations);

  for (size_t i = 1; i < sorted_final_indices.size(); ++i)
  {
    size_t start_index = sorted_final_indices[i - 1];
    size_t end_index = sorted_final_indices[i];

    MotionPrimitive primitive;
    primitive.type = MotionPrimitive::LINEAR_CARTESIAN;

    // Blend radius (zero at last point)
    if (i == sorted_final_indices.size() - 1)
    {
      primitive.blend_radius = 0.0;
    }
    else
    {
      const auto & p0 = trajectory[start_index].pose.position;
      const auto & p1 = trajectory[end_index].pose.position;
      const auto & p2 = trajectory[sorted_final_indices[i + 1]].pose.position;
      primitive.blend_radius =
        calculateBlendRadius({p0.x, p0.y, p0.z}, {p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z});
    }

    double velocity = -1.0;
    double acceleration = -1.0;
    double move_time = -1.0;

    if (use_time_not_vel_and_acc)
    {
      move_time = trajectory[end_index].time_from_start - trajectory[start_index].time_from_start;
      MotionArgument arg_time;
      arg_time.argument_name = "move_time";
      arg_time.argument_value = move_time;
      primitive.additional_arguments.push_back(arg_time);
    }
    else
    {
      // Use max velocity and acceleration in the reduced segment (min 0.01)
      double max_vel = 0.01;
      double max_acc = 0.01;
      for (size_t j = start_index + 1; j <= end_index && j - 1 < velocities.size(); ++j)
        max_vel = std::max(max_vel, velocities[j - 1]);
      for (size_t j = start_index + 2; j <= end_index && j - 2 < accelerations.size(); ++j)
        max_acc = std::max(max_acc, accelerations[j - 2]);
      velocity = max_vel;
      acceleration = max_acc;

      MotionArgument arg_vel;
      arg_vel.argument_name = "velocity";
      arg_vel.argument_value = velocity;
      primitive.additional_arguments.push_back(arg_vel);

      MotionArgument arg_acc;
      arg_acc.argument_name = "acceleration";
      arg_acc.argument_value = acceleration;
      primitive.additional_arguments.push_back(arg_acc);
    }

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

    std::cout << "Added LIN Primitive [" << i << "] (" << reason << "): (x,y,z,qx,qy,qz,qw) = ("
              << pose_stamped.pose.position.x << ", " << pose_stamped.pose.position.y << ", "
              << pose_stamped.pose.position.z << ", " << pose_stamped.pose.orientation.x << ", "
              << pose_stamped.pose.orientation.y << ", " << pose_stamped.pose.orientation.z << ", "
              << pose_stamped.pose.orientation.w << "), "
              << "blend_radius = " << primitive.blend_radius << ", "
              << "move_time = " << move_time << ", "
              << "velocity = " << velocity << ", "
              << "acceleration = " << acceleration << std::endl;
  }

  motion_sequence.motions = motion_primitives;

  return motion_sequence;
}

MotionSequence approxPtpPrimitivesWithRDP(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  double epsilon, bool use_time_not_vel_and_acc)
{
  MotionSequence motion_sequence;
  std::vector<MotionPrimitive> motion_primitives;

  if (trajectory.empty())
  {
    std::cerr << "[approxPtpPrimitivesWithRDP] Warning: trajectory is empty." << std::endl;
    return motion_sequence;
  }

  rdp::PointList points;
  for (const auto & pt : trajectory)
  {
    points.push_back(pt.joint_positions);
  }

  auto [reduced_points, reduced_indices] = rdp::rdpRecursive(points, epsilon);

  // Compute joint velocities and accelerations between trajectory points
  std::vector<double> joint_velocities, joint_accelerations;
  calculateJointVelAndAcc(trajectory, joint_velocities, joint_accelerations);

  for (size_t i = 1; i < reduced_points.size(); ++i)
  {
    MotionPrimitive primitive;
    primitive.type = MotionPrimitive::LINEAR_JOINT;

    if (i == reduced_points.size() - 1)
    {
      primitive.blend_radius = 0.0;
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

    double velocity = -1.0;
    double acceleration = -1.0;
    double move_time = -1.0;

    size_t start_index = reduced_indices[i - 1];
    size_t end_index = reduced_indices[i];

    if (use_time_not_vel_and_acc)
    {
      double prev_time = trajectory[start_index].time_from_start;
      double curr_time = trajectory[end_index].time_from_start;
      move_time = curr_time - prev_time;

      MotionArgument arg_time;
      arg_time.argument_name = "move_time";
      arg_time.argument_value = move_time;
      primitive.additional_arguments.push_back(arg_time);
    }
    else
    {
      // Get max velocity and acceleration in the reduced segment
      double max_vel = 0.0;
      double max_acc = 0.0;

      for (size_t j = start_index + 1; j <= end_index && j - 1 < joint_velocities.size(); ++j)
      {
        max_vel = std::max(max_vel, joint_velocities[j - 1]);
      }

      for (size_t j = start_index + 2; j <= end_index && j - 2 < joint_accelerations.size(); ++j)
      {
        max_acc = std::max(max_acc, joint_accelerations[j - 2]);
      }

      velocity = max_vel;
      acceleration = max_acc;

      MotionArgument arg_vel;
      arg_vel.argument_name = "velocity";
      arg_vel.argument_value = velocity;
      primitive.additional_arguments.push_back(arg_vel);

      MotionArgument arg_acc;
      arg_acc.argument_name = "acceleration";
      arg_acc.argument_value = acceleration;
      primitive.additional_arguments.push_back(arg_acc);
    }

    primitive.joint_positions = reduced_points[i];
    motion_primitives.push_back(primitive);

    std::cout << "Added PTP Primitive [" << i << "]: joints = (";
    for (size_t j = 0; j < reduced_points[i].size(); ++j)
    {
      std::cout << reduced_points[i][j];
      if (j + 1 < reduced_points[i].size()) std::cout << ", ";
    }
    std::cout << "), blend_radius = " << primitive.blend_radius << ", "
              << "move_time = " << move_time << ", "
              << "velocity = " << velocity << ", "
              << "acceleration = " << acceleration << std::endl;
  }

  motion_sequence.motions = motion_primitives;

  std::cout << "Reduced " << points.size() << " joint points to " << (reduced_points.size() - 1)
            << " PTP primitives with epsilon=" << epsilon << std::endl;

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

  // Clamp blend radius to [0, 0.1] with minimum threshold 0.001
  if (blend < 0.001)
  {
    blend = 0.0;
  }
  else if (blend > 0.1)
  {
    blend = 0.1;
  }

  return blend;
}

void calculateCartVelAndAcc(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  std::vector<double> & velocities, std::vector<double> & accelerations)
{
  velocities.clear();
  accelerations.clear();

  size_t num_points = trajectory.size();
  if (num_points < 2)
  {
    std::cerr << "[calculateCartVelAndAcc] Warning: trajectory too short to calculate "
                 "velocity/acceleration."
              << std::endl;
    return;
  }

  for (size_t i = 1; i < num_points; ++i)
  {
    const auto & previous_position = trajectory[i - 1].pose.position;
    const auto & current_position = trajectory[i].pose.position;

    double previous_time = trajectory[i - 1].time_from_start;
    double current_time = trajectory[i].time_from_start;
    double delta_time = current_time - previous_time;

    if (delta_time <= 0.0)
    {
      std::cerr << "[calculateCartVelAndAcc] Warning: non-positive time difference at index " << i
                << std::endl;
      velocities.push_back(0.0);
      continue;
    }

    double dx = current_position.x - previous_position.x;
    double dy = current_position.y - previous_position.y;
    double dz = current_position.z - previous_position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    double velocity = distance / delta_time;
    velocities.push_back(velocity);
  }

  // Accelerations (starts at index 1 because it compares velocities[i] and velocities[i - 1])
  for (size_t i = 1; i < velocities.size(); ++i)
  {
    double time_interval = trajectory[i + 1].time_from_start - trajectory[i].time_from_start;

    if (time_interval <= 0.0)
    {
      std::cerr << "[calculateCartVelAndAcc] Warning: non-positive time difference for "
                   "acceleration at index "
                << i << std::endl;
      accelerations.push_back(0.0);
      continue;
    }

    double delta_velocity = velocities[i] - velocities[i - 1];
    double acceleration = delta_velocity / time_interval;

    accelerations.push_back(std::abs(acceleration));
  }
}

void calculateJointVelAndAcc(
  const std::vector<approx_primitives_with_rdp::PlannedTrajectoryPoint> & trajectory,
  std::vector<double> & velocities, std::vector<double> & accelerations)
{
  velocities.clear();
  accelerations.clear();

  size_t num_points = trajectory.size();
  if (num_points < 2)
  {
    std::cerr << "[calculateJointVelAndAcc] Warning: trajectory too short to calculate "
                 "joint velocity/acceleration."
              << std::endl;
    return;
  }

  size_t num_joints = trajectory[0].joint_positions.size();

  // Compute joint velocities (max per timestep across joints)
  for (size_t i = 1; i < num_points; ++i)
  {
    double dt = trajectory[i].time_from_start - trajectory[i - 1].time_from_start;
    if (dt <= 0.0)
    {
      std::cerr << "[calculateJointVelAndAcc] Warning: non-positive time diff at index " << i
                << std::endl;
      velocities.push_back(0.0);
      continue;
    }

    double max_joint_vel = 0.0;
    for (size_t j = 0; j < num_joints; ++j)
    {
      double dq = trajectory[i].joint_positions[j] - trajectory[i - 1].joint_positions[j];
      double vel = std::abs(dq / dt);
      if (vel > max_joint_vel) max_joint_vel = vel;
    }

    velocities.push_back(max_joint_vel);
  }

  // Compute joint accelerations (max per timestep across joints)
  for (size_t i = 1; i < velocities.size(); ++i)
  {
    double dt = trajectory[i + 1].time_from_start - trajectory[i].time_from_start;
    if (dt <= 0.0)
    {
      std::cerr
        << "[calculateJointVelAndAcc] Warning: non-positive time diff for acceleration at index "
        << i << std::endl;
      accelerations.push_back(0.0);
      continue;
    }

    double dvel = velocities[i] - velocities[i - 1];
    accelerations.push_back(std::abs(dvel / dt));
  }
}

}  // namespace approx_primitives_with_rdp
