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

#ifndef JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_

#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "joint_trajectory_controller_parameters.hpp"

#include "rclcpp/node.hpp"

namespace joint_trajectory_controller
{
/**
 * \brief Trajectory state tolerances for position, velocity and acceleration variables.
 *
 * A tolerance value of zero means that no tolerance will be applied for that variable.
 */
struct StateTolerances
{
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

/**
 * \brief Trajectory segment tolerances.
 */
struct SegmentTolerances
{
  explicit SegmentTolerances(size_t size = 0) : state_tolerance(size), goal_state_tolerance(size) {}

  /** State tolerances that apply during segment execution. */
  std::vector<StateTolerances> state_tolerance;

  /** State tolerances that apply for the goal state only.*/
  std::vector<StateTolerances> goal_state_tolerance;

  /** Extra time after the segment end time allowed to reach the goal state tolerances. */
  double goal_time_tolerance = 0.0;
};

/**
 * \brief Populate trajectory segment tolerances using data from the ROS node.
 *
 * It is assumed that the following parameter structure is followed on the provided LifecycleNode.
 * Unspecified parameters will take the defaults shown in the comments:
 *
 * \code
 * constraints:
 *  goal_time: 1.0                   # Defaults to zero
 *  stopped_velocity_tolerance: 0.02 # Defaults to 0.01
 *  foo_joint:
 *    trajectory: 0.05               # Defaults to zero (ie. the tolerance is not enforced)
 *    goal:       0.03               # Defaults to zero (ie. the tolerance is not enforced)
 *  bar_joint:
 *    goal: 0.01
 * \endcode
 *
 * \param params The ROS Parameters
 * \return Trajectory segment tolerances.
 */
SegmentTolerances get_segment_tolerances(Params const & params)
{
  auto const & constraints = params.constraints;
  auto const n_joints = params.joints.size();

  SegmentTolerances tolerances;
  tolerances.goal_time_tolerance = constraints.goal_time;

  // State and goal state tolerances
  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);
  for (size_t i = 0; i < n_joints; ++i)
  {
    auto const joint = params.joints[i];
    tolerances.state_tolerance[i].position = constraints.joints_map.at(joint).trajectory;
    tolerances.goal_state_tolerance[i].position = constraints.joints_map.at(joint).goal;
    tolerances.goal_state_tolerance[i].velocity = constraints.stopped_velocity_tolerance;

    auto logger = rclcpp::get_logger("tolerance");
    RCLCPP_DEBUG(
      logger, "%s %f", (joint + ".trajectory").c_str(), tolerances.state_tolerance[i].position);
    RCLCPP_DEBUG(
      logger, "%s %f", (joint + ".goal").c_str(), tolerances.goal_state_tolerance[i].position);
  }

  return tolerances;
}

/**
 * \param state_error State error to check.
 * \param joint_idx Joint index for the state error
 * \param state_tolerance State tolerance of joint to check \p state_error against.
 * \param show_errors If the joint that violate its tolerance should be output to console. NOT REALTIME if true
 * \return True if \p state_error fulfills \p state_tolerance.
 */
inline bool check_state_tolerance_per_joint(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_error, int joint_idx,
  const StateTolerances & state_tolerance, bool show_errors = false)
{
  using std::abs;
  const double error_position = state_error.positions[joint_idx];
  const double error_velocity =
    state_error.velocities.empty() ? 0.0 : state_error.velocities[joint_idx];
  const double error_acceleration =
    state_error.accelerations.empty() ? 0.0 : state_error.accelerations[joint_idx];

  const bool is_valid =
    !(state_tolerance.position > 0.0 && abs(error_position) > state_tolerance.position) &&
    !(state_tolerance.velocity > 0.0 && abs(error_velocity) > state_tolerance.velocity) &&
    !(state_tolerance.acceleration > 0.0 && abs(error_acceleration) > state_tolerance.acceleration);

  if (is_valid)
  {
    return true;
  }

  if (show_errors)
  {
    const auto logger = rclcpp::get_logger("tolerances");
    RCLCPP_ERROR(logger, "Path state tolerances failed:");

    if (state_tolerance.position > 0.0 && abs(error_position) > state_tolerance.position)
    {
      RCLCPP_ERROR(
        logger, "Position Error: %f, Position Tolerance: %f", error_position,
        state_tolerance.position);
    }
    if (state_tolerance.velocity > 0.0 && abs(error_velocity) > state_tolerance.velocity)
    {
      RCLCPP_ERROR(
        logger, "Velocity Error: %f, Velocity Tolerance: %f", error_velocity,
        state_tolerance.velocity);
    }
    if (
      state_tolerance.acceleration > 0.0 && abs(error_acceleration) > state_tolerance.acceleration)
    {
      RCLCPP_ERROR(
        logger, "Acceleration Error: %f, Acceleration Tolerance: %f", error_acceleration,
        state_tolerance.acceleration);
    }
  }
  return false;
}

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
