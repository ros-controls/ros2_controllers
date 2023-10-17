// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/visibility_control.h"
#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
namespace joint_trajectory_controller
{
using TrajectoryPointIter = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  Trajectory();

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  explicit Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  explicit Trajectory(
    const rclcpp::Time & current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// Set the point before the trajectory message is replaced/appended
  /// Example: if we receive a new trajectory message and it's first point is 0.5 seconds
  /// from the current one, we call this function to log the current state, then
  /// append/replace the current trajectory
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void set_point_before_trajectory_msg(
    const rclcpp::Time & current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_point);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// Find the segment (made up of 2 points) and its expected state from the
  /// containing trajectory.
  /**
   * Sampling trajectory at given \p sample_time.
   * If position in the \p end_segment_itr is missing it will be deduced from provided velocity, or acceleration respectively.
   * Deduction assumes that the provided velocity or acceleration have to be reached at the time defined in the segment.
   *
   * Specific case returns for start_segment_itr and end_segment_itr:
   * - Sampling before the trajectory start:
   *   start_segment_itr = begin(), end_segment_itr = begin()
   * - Sampling exactly on a point of the trajectory:
   *    start_segment_itr = iterator where point is, end_segment_itr = iterator after start_segment_itr
   * - Sampling between points:
   *    start_segment_itr = iterator before the sampled point, end_segment_itr = iterator after start_segment_itr
   * - Sampling after entire trajectory:
   *    start_segment_itr = --end(), end_segment_itr = end()
   * - Sampling empty msg or before the time given in set_point_before_trajectory_msg()
   *    return false
   *
   * \param[in] sample_time Time at which trajectory will be sampled.
   * \param[in] interpolation_method Specify whether splines, another method, or no interpolation at all.
   * \param[out] expected_state Calculated new at \p sample_time.
   * \param[out] start_segment_itr Iterator to the start segment for given \p sample_time. See description above.
   * \param[out] end_segment_itr Iterator to the end segment for given \p sample_time. See description above.
   */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool sample(
    const rclcpp::Time & sample_time,
    const interpolation_methods::InterpolationMethod interpolation_method,
    trajectory_msgs::msg::JointTrajectoryPoint & output_state,
    TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr);

  /**
   * Do interpolation between 2 states given a time in between their respective timestamps
   *
   * The start and end states need not necessarily be specified all the way to the acceleration level:
   * - If only \b positions are specified, linear interpolation will be used.
   * - If \b positions and \b velocities are specified, a cubic spline will be used.
   * - If \b positions, \b velocities and \b accelerations are specified, a quintic spline will be used.
   *
   * If start and end states have different specifications
   * (eg. start is position-only, end is position-velocity), the lowest common specification will be used
   * (position-only in the example).
   *
   * \param[in] time_a Time at which the segment state equals \p state_a.
   * \param[in] state_a State at \p time_a.
   * \param[in] time_b Time at which the segment state equals \p state_b.
   * \param[in] state_b State at time \p time_b.
   * \param[in] sample_time The time to sample, between time_a and time_b.
   * \param[out] output The state at \p sample_time.
   */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void interpolate_between_points(
    const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
    const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
    const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output);

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter begin() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter end() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp::Time time_from_start() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool has_trajectory_msg() const;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> get_trajectory_msg() const
  {
    return trajectory_msg_;
  }

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp::Time get_trajectory_start_time() const { return trajectory_start_time_; }

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool is_sampled_already() const { return sampled_already_; }

private:
  void deduce_from_derivatives(
    trajectory_msgs::msg::JointTrajectoryPoint & first_state,
    trajectory_msgs::msg::JointTrajectoryPoint & second_state, const size_t dim,
    const double delta_t);

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  rclcpp::Time trajectory_start_time_;

  rclcpp::Time time_before_traj_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint state_before_traj_msg_;

  bool sampled_already_ = false;
};

/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  // t1 must be a subset of t2
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    else
    {
      const size_t t1_dist = std::distance(t1.begin(), t1_it);
      const size_t t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
