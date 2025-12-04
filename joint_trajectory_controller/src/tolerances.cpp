// Copyright 2013 PAL Robotics S.L.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// \author Adolfo Rodriguez Tsouroukdissian
/// \author Suryansh Singh (thedevmystic)

#include "joint_trajectory_controller/tolerances.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace joint_trajectory_controller
{

double resolve_tolerance_source(const double default_value, const double goal_value)
{
  // Erase value is -1.0
  const double ERASE_VALUE = -1.0;
  // Epsilon for comparision
  const double EPSILON = std::numeric_limits<double>::epsilon();

  if (goal_value > 0.0)
  {
    return goal_value;
  }
  else if (std::abs(goal_value - ERASE_VALUE) < EPSILON)
  {
    return 0.0;
  }
  else if (goal_value < 0.0)
  {
    throw std::runtime_error("Illegal tolerance value");
  }
  
  // goal_value is 0.0, use default
  return default_value;
}

SegmentTolerances get_segment_tolerances(rclcpp::Logger & jtc_logger, const Params & params)
{
  auto const & constraints = params.constraints;
  auto const n_joints = params.joints.size();

  SegmentTolerances tolerances;
  tolerances.goal_time_tolerance = constraints.goal_time;
  static auto logger = jtc_logger.get_child("param_tolerance_loader");
  RCLCPP_DEBUG(logger, "goal_time %f", constraints.goal_time);

  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);

  for (size_t i = 0; i < n_joints; ++i)
  {
    auto const joint = params.joints[i];
    auto const & joint_constraints = constraints.joints_map.at(joint);
    
    // Path/Trajectory Tolerances (state_tolerance)
    tolerances.state_tolerance[i].position = joint_constraints.trajectory.position;
    tolerances.state_tolerance[i].velocity = joint_constraints.trajectory.velocity;
    tolerances.state_tolerance[i].acceleration = joint_constraints.trajectory.acceleration;

    // Goal Tolerances (goal_state_tolerance)
    tolerances.goal_state_tolerance[i].position = joint_constraints.goal.position;

    // Velocity tolerance must be at least the global stopped_velocity_tolerance
    tolerances.goal_state_tolerance[i].velocity = std::max(
      joint_constraints.goal.velocity,
      constraints.stopped_velocity_tolerance
    );

    tolerances.goal_state_tolerance[i].acceleration = joint_constraints.goal.acceleration;

    // Debug Logging
    RCLCPP_DEBUG(logger, "--- Tolerances for Joint: %s ---", joint.c_str());
    RCLCPP_DEBUG(logger, "Path Pos/Vel/Acc: %f / %f / %f",
      tolerances.state_tolerance[i].position,
      tolerances.state_tolerance[i].velocity,
      tolerances.state_tolerance[i].acceleration);
    RCLCPP_DEBUG(logger, "Goal Pos/Vel/Acc: %f / %f / %f",
      tolerances.goal_state_tolerance[i].position,
      tolerances.goal_state_tolerance[i].velocity,
      tolerances.goal_state_tolerance[i].acceleration);
  }

  return tolerances;
}

SegmentTolerances get_segment_tolerances(
  rclcpp::Logger & jtc_logger, const SegmentTolerances & default_tolerances,
  const control_msgs::action::FollowJointTrajectory::Goal & goal,
  const std::vector<std::string> & joints)
{
  SegmentTolerances active_tolerances = default_tolerances;
  static auto logger = jtc_logger.get_child("goal_tolerance_loader");
  
  // Create a map to look up joint index by name for fast access
  std::map<std::string, size_t> joint_to_id;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    joint_to_id[joints[i]] = i;
  }

  // Process goal.path_tolerance (Execution Constraints)
  for (const auto & joint_tol : goal.path_tolerance)
  {
    auto it = joint_to_id.find(joint_tol.name);
    if (it == joint_to_id.end())
    {
      RCLCPP_ERROR(
        logger,
        "Path tolerance specified for unknown joint '%s'. Using default tolerances.",
        joint_tol.name.c_str());
      return default_tolerances;
    }
    const size_t i = it->second;
    std::string interface = "position";

    try
    {
      // Position
      active_tolerances.state_tolerance[i].position = resolve_tolerance_source(
        default_tolerances.state_tolerance[i].position, joint_tol.position);

      // Velocity
      interface = "velocity";
      active_tolerances.state_tolerance[i].velocity = resolve_tolerance_source(
        default_tolerances.state_tolerance[i].velocity, joint_tol.velocity);

      // Acceleration
      interface = "acceleration";
      active_tolerances.state_tolerance[i].acceleration = resolve_tolerance_source(
        default_tolerances.state_tolerance[i].acceleration, joint_tol.acceleration);
    }
    catch (const std::runtime_error & ex)
    {
      RCLCPP_ERROR(
        logger,
        "Illegal path tolerance value for joint '%s' and interface '%s': %s. Using default tolerances.",
        joint_tol.name.c_str(), interface.c_str(), ex.what());
      return default_tolerances;
    }
  }

  // Process goal.goal_tolerance (Endpoint Constraints)
  for (const auto & goal_tol : goal.goal_tolerance)
  {
    auto it = joint_to_id.find(goal_tol.name);
    if (it == joint_to_id.end())
    {
      RCLCPP_ERROR(
        logger,
        "Goal tolerance specified for unknown joint '%s'. Using default tolerances.",
        goal_tol.name.c_str());
      return default_tolerances;
    }
    const size_t i = it->second;
    std::string interface = "position";

    try
    {
      // Position
      active_tolerances.goal_state_tolerance[i].position = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].position, goal_tol.position);

      // Velocity
      interface = "velocity";
      active_tolerances.goal_state_tolerance[i].velocity = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].velocity, goal_tol.velocity);

      // Acceleration
      interface = "acceleration";
      active_tolerances.goal_state_tolerance[i].acceleration = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].acceleration, goal_tol.acceleration);
    }
    catch (const std::runtime_error & ex)
    {
      RCLCPP_ERROR(
        logger,
        "Illegal goal tolerance value for joint '%s' and interface '%s': %s. Using default tolerances.",
        goal_tol.name.c_str(), interface.c_str(), ex.what());
      return default_tolerances;
    }
  }

  // Process goal_time_tolerance
  if (goal.goal_time_tolerance.sec != 0 || goal.goal_time_tolerance.nanosec != 0)
  {
    const double goal_time_sec =
      static_cast<double>(goal.goal_time_tolerance.sec) +
      static_cast<double>(goal.goal_time_tolerance.nanosec) / 1e9;

    active_tolerances.goal_time_tolerance = std::max(0.0, goal_time_sec);
  }

  return active_tolerances;
}

}  // namespace joint_trajectory_controller
