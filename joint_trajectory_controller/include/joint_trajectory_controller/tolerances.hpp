///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
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
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_

// C++ standard
#include <cassert>
#include <cmath>
#include <string>
#include <vector>

// ROS
#include "rclcpp/node.hpp"

// ROS messages
#include "control_msgs/action/follow_joint_trajectory.hpp"

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
  explicit SegmentTolerances(size_t size = 0)
  : state_tolerance(size),
    goal_state_tolerance(size)
  {}

  /** State tolerances that apply during segment execution. */
  std::vector<StateTolerances> state_tolerance;

  /** State tolerances that apply for the goal state only.*/
  std::vector<StateTolerances> goal_state_tolerance;

  /** Extra time after the segment end time allowed to reach the goal state tolerances. */
  double goal_time_tolerance = 0.0;
};

void declareSegmentTolerances(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
  node->declare_parameter<double>("constraints.goal_time", 0.0);
  // not possible to declare per-joint tolerances since we need to obtain joint_names
}

/**
 * \brief Populate trajectory segment tolerances from data in the ROS parameter server.
 *
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Unspecified parameters
 * will take the defaults shown in the comments:
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
 * \param nh NodeHandle where the tolerances are specified.
 * \param joint_names Names of joints to look for in the parameter server for a tolerance specification.
 * \return Trajectory segment tolerances.
 */
SegmentTolerances getSegmentTolerances(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::vector<std::string> & joint_names)
{
  const unsigned int n_joints = joint_names.size();
  SegmentTolerances tolerances;

  // State and goal state tolerances
  double stopped_velocity_tolerance;
  node->get_parameter_or<double>("constraints.stopped_velocity_tolerance",
    stopped_velocity_tolerance, 0.01);

  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);
  for (auto i = 0u; i < n_joints; ++i) {
    std::string prefix = "constraints." + joint_names[i];

    node->get_parameter_or<double>(prefix + ".trajectory", tolerances.state_tolerance[i].position,
      0.0);
    node->get_parameter_or<double>(prefix + ".goal", tolerances.goal_state_tolerance[i].position,
      0.0);
    RCLCPP_DEBUG(rclcpp::get_logger("tolerance"), "%s %f",
      (prefix + ".trajectory").c_str(), tolerances.state_tolerance[i].position);
    RCLCPP_DEBUG(rclcpp::get_logger("tolerance"), "%s %f",
      (prefix + ".goal").c_str(), tolerances.goal_state_tolerance[i].position);

    tolerances.goal_state_tolerance[i].velocity = stopped_velocity_tolerance;
  }

  // Goal time tolerance
  node->get_parameter_or<double>("constraints.goal_time", tolerances.goal_time_tolerance, 0.0);

  return tolerances;
}

/**
 * \param state_error State error to check.
 * \param joint_idx Joint index for the state error
 * \param state_tolerance State tolerance of joint to check \p state_error against.
 * \param show_errors If the joint that violate its tolerance should be output to console. NOT REALTIME if true
 * \return True if \p state_error fulfills \p state_tolerance.
 */
inline bool checkStateTolerancePerJoint(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_error,
  int joint_idx,
  const StateTolerances & state_tolerance,
  bool show_errors = false)
{
  using std::abs;
  double error_position = state_error.positions[joint_idx];
  double error_velocity = state_error.velocities[joint_idx];
  double error_acceleration = state_error.accelerations[joint_idx];

  const bool is_valid =
    !(state_tolerance.position > 0.0 && abs(error_position) > state_tolerance.position) &&
    !(state_tolerance.velocity > 0.0 && abs(error_velocity) > state_tolerance.velocity) &&
    !(state_tolerance.acceleration > 0.0 && abs(error_acceleration) > state_tolerance.acceleration);

  if (!is_valid) {
    if (show_errors) {
      auto logger = rclcpp::get_logger("tolerances");
      RCLCPP_ERROR_STREAM(logger, "Path state tolerances failed:");

      if (state_tolerance.position > 0.0 && abs(error_position) > state_tolerance.position) {
        RCLCPP_ERROR_STREAM(logger, "Position Error: " << error_position <<
          " Position Tolerance: " << state_tolerance.position);
      }
      if (state_tolerance.velocity > 0.0 && abs(error_velocity) > state_tolerance.velocity) {
        RCLCPP_ERROR_STREAM(logger, "Velocity Error: " << error_velocity <<
          " Velocity Tolerance: " << state_tolerance.velocity);
      }
      if (state_tolerance.acceleration > 0.0 &&
        abs(error_acceleration) > state_tolerance.acceleration)
      {
        RCLCPP_ERROR_STREAM(logger, "Acceleration Error: " << error_acceleration <<
          " Acceleration Tolerance: " << state_tolerance.acceleration);
      }
    }
    return false;
  }
  return true;
}

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
