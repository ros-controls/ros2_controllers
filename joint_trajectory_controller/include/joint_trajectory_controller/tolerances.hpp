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

#ifndef JOINT_TRAJECTORY_CONTROLLER_TOLERANCES_H
#define JOINT_TRAJECTORY_CONTROLLER_TOLERANCES_H

// C++ standard
#include <cassert>
#include <cmath>
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

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
  SegmentTolerances(size_t size = 0)
    : state_tolerance(size),
      goal_state_tolerance(size),
      goal_time_tolerance(0.0)
  {}

  /** State tolerances that appply during segment execution. */
  std::vector<StateTolerances> state_tolerance;

  /** State tolerances that apply for the goal state only.*/
  std::vector<StateTolerances> goal_state_tolerance;

  /** Extra time after the segment end time allowed to reach the goal state tolerances. */
  double goal_time_tolerance;

};

void declareSegmentTolerances(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
  node->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
  node->declare_parameter<double>("constraints.goal_time", 0.0);
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
SegmentTolerances getSegmentTolerances(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                       const std::vector<std::string>& joint_names)
{
  const unsigned int n_joints = joint_names.size();
  SegmentTolerances tolerances;

  // State and goal state tolerances
  double stopped_velocity_tolerance;
  node->get_parameter_or<double>("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance, 0.01);
  
  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    std::string prefix = "constraints." + joint_names[i];
    // not possible to declare in init function since we need joint_names to be set first
    //node->declare_parameter<double>(prefix + ".trajectory", 0.0);
    //node->declare_parameter<double>(prefix + ".goal", 0.0);

    node->get_parameter_or<double>(prefix + ".trajectory", tolerances.state_tolerance[i].position, 0.0);
    node->get_parameter_or<double>(prefix + ".goal", tolerances.goal_state_tolerance[i].position, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("tolerance"), "%s %f", (prefix + ".trajectory").c_str(), tolerances.state_tolerance[i].position);
    RCLCPP_INFO(rclcpp::get_logger("tolerance"), "%s %f", (prefix + ".goal").c_str(), tolerances.goal_state_tolerance[i].position);
    
    tolerances.goal_state_tolerance[i].velocity = stopped_velocity_tolerance;
  }

  // Goal time tolerance
  node->get_parameter_or<double>("constraints.goal_time", tolerances.goal_time_tolerance, 0.0);

  return tolerances;
}

/**
 * \param state_error State error to check.
 * \param state_tolerance State tolerances to check \p state_error against.
 * \param show_errors If the joints that violate their tolerances should be output to console. NOT REALTIME if true
 * \return True if \p state_error fulfills \p state_tolerance.
 */
template <class State>
inline bool checkStateTolerance(const State&                     state_error,
                                const std::vector<StateTolerances>& state_tolerance,
                                bool show_errors = false)
{
  const unsigned int n_joints = state_tolerance.size();

  // Preconditions
  assert(n_joints == state_error.position.size());
  assert(n_joints == state_error.velocity.size());
  assert(n_joints == state_error.acceleration.size());

  for (unsigned int i = 0; i < n_joints; ++i)
  {
    using std::abs;
    const StateTolerances& tol = state_tolerance[i]; // Alias for brevity
    const bool is_valid = !(tol.position     > 0.0 && abs(state_error.position[i])     > tol.position) &&
                          !(tol.velocity     > 0.0 && abs(state_error.velocity[i])     > tol.velocity) &&
                          !(tol.acceleration > 0.0 && abs(state_error.acceleration[i]) > tol.acceleration);

    if (!is_valid)
    {
      if( show_errors )
      {
        auto logger = rclcpp::get_logger("tolerances");
        RCLCPP_ERROR_STREAM(logger, "Path state tolerances failed on joint " << i);

        if (tol.position     > 0.0 && abs(state_error.position[i])     > tol.position)
          RCLCPP_ERROR_STREAM(logger, "Position Error: " << state_error.position[i] <<
            " Position Tolerance: " << tol.position);
        if (tol.velocity     > 0.0 && abs(state_error.velocity[i])     > tol.velocity)
          RCLCPP_ERROR_STREAM(logger, "Velocity Error: " << state_error.velocity[i] <<
            " Velocity Tolerance: " << tol.velocity);
        if (tol.acceleration > 0.0 && abs(state_error.acceleration[i]) > tol.acceleration)
          RCLCPP_ERROR_STREAM(logger, "Acceleration Error: " << state_error.acceleration[i] <<
            " Acceleration Tolerance: " << tol.acceleration);
      }
      return false;
    }
  }
  return true;
}

/**
 * \param state_error State error to check.
 * \param state_tolerance State tolerances to check \p state_error against.
 * \param show_errors If the joint that violate its tolerance should be output to console. NOT REALTIME if true
 * \return True if \p state_error fulfills \p state_tolerance.
 */
inline bool checkStateTolerancePerJoint(const trajectory_msgs::msg::JointTrajectoryPoint& state_error,
                                        int joint_idx,
                                        const StateTolerances& state_tolerance,
                                        bool show_errors = false)
{

  using std::abs;
  double error_position = state_error.positions[joint_idx];
  double error_velocity = state_error.velocities[joint_idx];
  double error_acceleration = state_error.accelerations[joint_idx];

  const bool is_valid = !(state_tolerance.position     > 0.0 && abs(error_position)     > state_tolerance.position) &&
                        !(state_tolerance.velocity     > 0.0 && abs(error_velocity)     > state_tolerance.velocity) &&
                        !(state_tolerance.acceleration > 0.0 && abs(error_acceleration) > state_tolerance.acceleration);

  if (!is_valid)
  {
    if( show_errors )
    {
      auto logger = rclcpp::get_logger("tolerances");
      RCLCPP_ERROR_STREAM(logger, "Path state tolerances failed:");

      if (state_tolerance.position     > 0.0 && abs(error_position)     > state_tolerance.position)
        RCLCPP_ERROR_STREAM(logger, "Position Error: " << error_position <<
          " Position Tolerance: " << state_tolerance.position);
      if (state_tolerance.velocity     > 0.0 && abs(error_velocity)     > state_tolerance.velocity)
        RCLCPP_ERROR_STREAM(logger, "Velocity Error: " << error_velocity <<
          " Velocity Tolerance: " << state_tolerance.velocity);
      if (state_tolerance.acceleration > 0.0 && abs(error_acceleration) > state_tolerance.acceleration)
        RCLCPP_ERROR_STREAM(logger, "Acceleration Error: " << error_acceleration <<
          " Acceleration Tolerance: " << state_tolerance.acceleration);
    }
    return false;
  }
  return true;
}

/**
 * \brief Update data in \p tols from data in \p msg_tol.
 *
 * - If a value in \p tol_msg is positive, the corresponding values in \p tols will be overritten.
 * - If a value in \p tol_msg is negative, the corresponding values in \p tols will be reset.
 * - If a value in \p tol_msg is zero, the corresponding values in \p tols is unchanged.
 *
 * \param[in] tol_msg Message containing tolerance values \p tols will be updated with.
 * \param[out] tols Tolerances values to update.
 *
 **/
void updateStateTolerances(const control_msgs::msg::JointTolerance& tol_msg, StateTolerances& tols)
{
  if      (tol_msg.position     > 0.0) {tols.position     = static_cast<double>(tol_msg.position);}
  else if (tol_msg.position     < 0.0) {tols.position     = 0.0;}

  if      (tol_msg.velocity     > 0.0) {tols.velocity     = static_cast<double>(tol_msg.velocity);}
  else if (tol_msg.velocity     < 0.0) {tols.velocity     = 0.0;}

  if      (tol_msg.acceleration > 0.0) {tols.acceleration = static_cast<double>(tol_msg.acceleration);}
  else if (tol_msg.acceleration < 0.0) {tols.acceleration = 0.0;}
}

/**
 * \brief Update data in \p tols from data in \p goal.
 *
 * \param[in] goal Action goal data containing tolerance values \p tols will be updated with.
 * \param[in] joint_names Names of joints in \p tols, with the same ordering.
 * \param[out] tols Tolerances values to update.
 */
void updateSegmentTolerances(const control_msgs::action::FollowJointTrajectory::Goal& goal,
                             const std::vector<std::string>& joint_names,
                             SegmentTolerances& tols
)
{
  // Preconditions
  assert(joint_names.size() == tols.state_tolerance.size());
  assert(joint_names.size() == tols.goal_state_tolerance.size());

  typedef typename std::vector<std::string>::const_iterator                  StringConstIterator;
  typedef typename std::vector<control_msgs::msg::JointTolerance>::const_iterator TolMsgConstIterator;

  for (StringConstIterator names_it = joint_names.begin(); names_it != joint_names.end(); ++names_it)
  {
    const typename std::vector<std::string>::size_type id = std::distance(joint_names.begin(), names_it);

    // Update path tolerances
    const std::vector<control_msgs::msg::JointTolerance>& state_tol = goal.path_tolerance;
    for(TolMsgConstIterator state_tol_it = state_tol.begin(); state_tol_it != state_tol.end(); ++state_tol_it)
    {
      if (*names_it == state_tol_it->name) {updateStateTolerances(*state_tol_it, tols.state_tolerance[id]);}
    }

    // Update goal state tolerances
    const std::vector<control_msgs::msg::JointTolerance>& g_state_tol = goal.goal_tolerance;
    for(TolMsgConstIterator g_state_tol_it = g_state_tol.begin(); g_state_tol_it != g_state_tol.end(); ++g_state_tol_it)
    {
      if (*names_it == g_state_tol_it->name) {updateStateTolerances(*g_state_tol_it, tols.goal_state_tolerance[id]);}
    }
  }

  // Update goal time tolerance
  const rclcpp::Duration& goal_time_tolerance = goal.goal_time_tolerance;
  if      (goal_time_tolerance < rclcpp::Duration(0.0)) {tols.goal_time_tolerance = 0.0;}
  else if (goal_time_tolerance > rclcpp::Duration(0.0)) {tols.goal_time_tolerance = goal_time_tolerance.seconds();}
}

} // namespace

#endif // header guard
