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

#ifndef JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller_parameters.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

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
 * \brief Trajectory segment tolerances for validation during execution and at the endpoint.
 *
 * This structure defines the allowable deviation from the planned trajectory
 * during execution (state_tolerance) and the final required accuracy at the
 * segment's endpoint (goal_state_tolerance).
 */
struct SegmentTolerances
{
  explicit SegmentTolerances(size_t size = 0) : state_tolerance(size), goal_state_tolerance(size) {}

  /**
   * \brief State tolerances applied throughout the segment's execution.
   *
   * If the actual joint state deviates from the desired setpoint by more than
   * this tolerance at any time during the segment, the trajectory execution
   * may be aborted. This vector contains one entry for each joint.
   */
  std::vector<StateTolerances> state_tolerance;

  /**
   * \brief State tolerances applied only to the final goal state.
   *
   * These are the tolerances the joint state must meet at the segment's
   * end time (or within the goal_time_tolerance window) for the segment
   * to be considered successfully completed. This vector contains one entry
   * for each joint.
   */
  std::vector<StateTolerances> goal_state_tolerance;

  /**
   * \brief Extra time allowed to reach the goal state tolerances.
   *
   * This defines a time window (in seconds) after the segment's
   * planned end time, during which the goal_state_tolerance must be met.
   * This allows the robot to "settle" into the final target position.
   */
  double goal_time_tolerance = 0.0;
};

/**
 * \brief Resolves the final effective tolerance value based on provided goal tolerance.
 *
 * This function applies logic based on conventions of ROS joint tolerance messages
 * (like control_msgs/msg/JointTolerance.msg) where special negative values are used
 * to modify or clear the default tolerance.
 *
 * The logic applied is:
 * 1. Positive value: The goal tolerance is explicitly used.
 * 2. ERASE_VALUE (-1.0): The tolerance is cleared/erased, returning 0.0, which means the
 *                        tolerance check is disabled or the joint is unrestricted.
 * 3. Zero (0.0): The goal tolerance is unspecified, and the default value is returned.
 * 4. Other negative values (except ERASE_VALUE): Illegal input, throws an error.
 *
 * \param[in] default_value The pre-configured or default tolerance value.
 * \param[in] goal_value The tolerance value specified in the goal message.
 * \return The resolved tolerance value to be used for validation.
 * \throws std::runtime_error If an illegal negative tolerance value is provided.
 */
double resolve_tolerance_source(const double default_value, const double goal_value);

/**
 * \brief Populate trajectory segment tolerances using data from the ROS node.
 *
 * It is assumed that the following parameter structure is followed on the provided LifecycleNode.
 * Unspecified parameters will take the defaults shown below:
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
 * The `stopped_velocity_tolerance` is applied as the minimum velocity tolerance
 * for the goal state of *all* joints, overriding individual `joint.goal.velocity`
 * settings if the global value is higher.
 *
 * \param[in] jtc_logger The logger to use for output.
 * \param[in] params The ROS Parameters structure containing joint lists and constraints.
 * \return Trajectory segment tolerances populated from parameters.
 */
SegmentTolerances get_segment_tolerances(rclcpp::Logger & jtc_logger, const Params & params);

/**
 * \brief Populate trajectory segment tolerances using data from an action goal.
 *
 * This function creates a new `SegmentTolerances` object by starting with a copy of
 * the `default_tolerances` and then overriding specific values with tolerances
 * provided in the `FollowJointTrajectory` action goal.
 *
 * The `resolve_tolerance_source` function is used to correctly handle special
 * tolerance values (0.0 for default, -1.0 for erase). If any provided joint name
 * is invalid or a tolerance value is illegal, the function immediately returns the
 * unmodified `default_tolerances`.
 *
 * \param[in] jtc_logger The logger to use for output and debugging messages.
 * \param[in] default_tolerances The base tolerances configured via ROS parameters, used if
 *                               the action goal does not specify any, or if the goal is invalid.
 * \param[in] goal The new `FollowJointTrajectory` action goal containing optional tolerance
 * overrides in `path_tolerance` and `goal_tolerance` fields.
 * \param[in] joints The list of joints configured for the controller (used for index mapping).
 * \return The resolved `SegmentTolerances` object, prioritizing goal values over defaults.
 */
SegmentTolerances get_segment_tolerances(
  rclcpp::Logger & jtc_logger, const SegmentTolerances & default_tolerances,
  const control_msgs::action::FollowJointTrajectory::Goal & goal,
  const std::vector<std::string> & joints);

/**
 * \brief Calculates the error point (desired - actual) for a trajectory point.
 *
 * \param[in] desired_state The commanded state point.
 * \param[in] current_state The actual state point from the hardware.
 * \param[in] is_wraparounds A vector indicating which joints are wraparound (e.g., continuous).
 * \param[in] show_errors If true, log messages about size mismatches will be printed.
 * \return A JointTrajectoryPoint where positions, velocities, etc., are the difference.
 */
trajectory_msgs::msg::JointTrajectoryPoint compute_error_trajectory_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & desired_state,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state,
  const std::vector<bool> & is_wraparounds, bool show_errors = false);

/**
 * \brief Checks if the error for a single joint state component is within the defined tolerance.
 *
 * This function is used to validate the state error (e.g., actual minus desired) for a
 * specific joint against its maximum allowable deviations (tolerance) in position, velocity, and
 * acceleration. A tolerance value of 0.0 means that the corresponding check is skipped.
 * The check is successful only if the absolute error is less than or equal to the tolerance
 * for all components that have a positive tolerance set.
 *
 * \param[in] state_error The difference (error) between the actual and desired joint state.
 * \param[in] joint_idx The index of the joint in the `state_error` message to be checked.
 * \param[in] state_tolerance The tolerance structure defining the maximum allowed absolute errors.
 * \param[in] show_errors If true, log messages about the tolerance violation will be printed.
 *
 * **WARNING: Logging is not real-time safe.**
 *
 * \return True if the absolute error for all checked components (where tolerance > 0.0)
 * is less than or equal to the respective tolerance; False otherwise.
 */
bool check_state_tolerance_per_joint(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_error, size_t joint_idx,
  const StateTolerances & state_tolerance, bool show_errors = false);

/**
 * \brief Checks if the entire trajectory point error is within the segment tolerances.
 *
 * This function iterates over all joints and calls check_state_tolerance_per_joint().
 *
 * \param[in] state_error Error point (difference between commanded and actual) for all joints.
 * \param[in] segment_tolerances StateTolerances for all joints in the trajectory.
 * \param[in] show_errors If true, the joint that violates its tolerance should be output to
 *                        console. (NON-REALTIME if true).
 * \return True if ALL joints are within their defined tolerances, false otherwise.
 */
bool check_trajectory_point_tolerance(
  const trajectory_msgs::msg::JointTrajectoryPoint & state_error,
  const std::vector<StateTolerances> & segment_tolerances, bool show_errors = false);

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TOLERANCES_HPP_
