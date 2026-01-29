// Copyright 2013 PAL Robotics S.L.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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
  constexpr double ERASE_VALUE = -1.0;
  constexpr double EPSILON = std::numeric_limits<double>::epsilon();

  // Helper lambda to check for values
  auto is_erase_value = [&](double value) { return std::fabs(value - ERASE_VALUE) < EPSILON; };

  if (goal_value > 0.0)
  {
    // Valid +ve tolerance, return by value
    return goal_value;
  }
  else if (is_erase_value(goal_value))
  {
    // Valid -ve tolerance (-1), erase value to 0
    return 0.0;
  }
  else if (goal_value < 0.0)
  {
    // All other -ve tolerances are invalid.
    throw std::runtime_error("Illegal tolerance value.");
  }
  // Return default value if don't pass any check
  return default_value;
}

SegmentTolerances get_segment_tolerances(rclcpp::Logger & jtc_logger, const Params & params)
{
  auto const & constraints = params.constraints;
  auto const n_joints = params.joints.size();

  SegmentTolerances tolerances;
  tolerances.goal_time_tolerance = constraints.goal_time;
  static auto logger = jtc_logger.get_child("tolerance");
  RCLCPP_DEBUG(logger, "Goal Time: %f", constraints.goal_time);

  // State and goal tolerances
  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);
  for (size_t i = 0; i < n_joints; ++i)
  {
    auto const joint = params.joints[i];
    auto const & joint_constraints = constraints.joints_map.at(joint);

    // Tolerances
    tolerances.state_tolerance[i].position = joint_constraints.trajectory;
    tolerances.goal_state_tolerance[i].position = joint_constraints.goal;
    tolerances.goal_state_tolerance[i].velocity = constraints.stopped_velocity_tolerance;

    // Debug Logging
    RCLCPP_DEBUG(logger, "--- Tolerances for Joint: %s ---", joint.c_str());
    RCLCPP_DEBUG(logger, "Trajectory Position: %f", tolerances.state_tolerance[i].position);
    RCLCPP_DEBUG(logger, "Goal Position: %f", tolerances.goal_state_tolerance[i].position);
    RCLCPP_DEBUG(logger, "Goal Velocity: %f", tolerances.goal_state_tolerance[i].velocity);
    RCLCPP_DEBUG(logger, "---------------------------");
  }

  return tolerances;
}

SegmentTolerances get_segment_tolerances(
  rclcpp::Logger & jtc_logger, const SegmentTolerances & default_tolerances,
  const control_msgs::action::FollowJointTrajectory::Goal & goal,
  const std::vector<std::string> & joints)
{
  static auto logger = jtc_logger.get_child("tolerance");
  SegmentTolerances active_tolerances(default_tolerances);

  // Create a map to look up joint by name for fast access
  std::map<std::string, size_t> joint_to_id;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    joint_to_id[joints[i]] = i;
  }

  // Process goal_time_tolerance
  try
  {
    double goal_time_sec = rclcpp::Duration(goal.goal_time_tolerance).seconds();
    active_tolerances.goal_time_tolerance =
      resolve_tolerance_source(default_tolerances.goal_time_tolerance, goal_time_sec);
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_WARN(
      logger, "Illegal goal time tolerance specified: %f. Using default tolerances.",
      rclcpp::Duration(goal.goal_time_tolerance).seconds());
    return default_tolerances;
  }

  // Process goal.path_tolerance (Execution Constraints)
  for (const auto & joint_tol : goal.path_tolerance)
  {
    auto it = joint_to_id.find(joint_tol.name);
    if (it == joint_to_id.end())
    {
      RCLCPP_WARN(
        logger, "Path tolerance specified for unknown joint '%s'. Using default tolerances.",
        joint_tol.name.c_str());
      return default_tolerances;
    }
    size_t i = it->second;
    std::string interface = "";  // For error tracking

    try
    {
      // Position
      interface = "position";
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
    catch (const std::runtime_error & e)
    {
      RCLCPP_WARN(
        logger,
        "Illegal %s path tolerance value specified for joint '%s'. Using default tolerances.",
        interface.c_str(), joint_tol.name.c_str());
      return default_tolerances;
    }
  }

  // Process goal.goal_tolerance (Goal/Endpoint Constraints)
  for (const auto & joint_tol : goal.goal_tolerance)
  {
    auto it = joint_to_id.find(joint_tol.name);
    if (it == joint_to_id.end())
    {
      RCLCPP_WARN(
        logger, "Goal tolerance specified for unknown joint '%s'. Using default tolerances.",
        joint_tol.name.c_str());
      return default_tolerances;
    }
    size_t i = it->second;
    std::string interface = "";  // For error tracking

    try
    {
      // Position
      interface = "position";
      active_tolerances.goal_state_tolerance[i].position = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].position, joint_tol.position);

      // Velocity
      interface = "velocity";
      active_tolerances.goal_state_tolerance[i].velocity = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].velocity, joint_tol.velocity);

      // Acceleration
      interface = "acceleration";
      active_tolerances.goal_state_tolerance[i].acceleration = resolve_tolerance_source(
        default_tolerances.goal_state_tolerance[i].acceleration, joint_tol.acceleration);
    }
    catch (const std::runtime_error & e)
    {
      RCLCPP_WARN(
        logger,
        "Illegal %s goal tolerance value specified for joint '%s'. Using default tolerances.",
        interface.c_str(), joint_tol.name.c_str());
      return default_tolerances;
    }
  }

  // Debug Logging
  RCLCPP_DEBUG(logger, "---- Active Tolerances ----");
  RCLCPP_DEBUG(logger, "Goal Time: %f", active_tolerances.goal_time_tolerance);
  for (size_t i = 0; i < joints.size(); ++i)
  {
    RCLCPP_DEBUG(logger, "--- Tolerances for Joint: %s ---", joints[i].c_str());
    RCLCPP_DEBUG(logger, "Trajectory Position: %f", active_tolerances.state_tolerance[i].position);
    RCLCPP_DEBUG(logger, "Trajectory Velocity: %f", active_tolerances.state_tolerance[i].velocity);
    RCLCPP_DEBUG(
      logger, "Trajectory Acceleration: %f", active_tolerances.state_tolerance[i].acceleration);
    RCLCPP_DEBUG(logger, "Goal Position: %f", active_tolerances.goal_state_tolerance[i].position);
    RCLCPP_DEBUG(logger, "Goal Velocity: %f", active_tolerances.goal_state_tolerance[i].velocity);
    RCLCPP_DEBUG(
      logger, "Goal Acceleration: %f", active_tolerances.goal_state_tolerance[i].acceleration);
  }
  RCLCPP_DEBUG(logger, "---------------------------");

  return active_tolerances;
}

}  // namespace joint_trajectory_controller
