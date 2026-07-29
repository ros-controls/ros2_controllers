// Copyright (c) 2026 ros2_control Development Team
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

#ifndef CARTESIAN_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_HPP_
#define CARTESIAN_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_HPP_

#include <cstddef>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace cartesian_trajectory_controller
{

/// Flip quaternion signs so consecutive waypoints lie on the same hemisphere, making SLERP take the
/// shortest arc across every segment (the quaternion analog of angles::shortest_angular_distance).
void align_quaternions_shortest_arc(std::vector<Eigen::Quaterniond> & orientations);

/// Time to move between two poses without exceeding the Cartesian or angular speed, floored at
/// min_duration so coincident poses still advance in time. Used to synthesize waypoint timing when
/// the incoming chunk carries none.
double min_segment_duration(
  const Eigen::Vector3d & from_position, const Eigen::Quaterniond & from_orientation,
  const Eigen::Vector3d & to_position, const Eigen::Quaterniond & to_orientation,
  double max_linear_speed, double max_angular_speed, double min_duration);

/// Time-parameterized Cartesian path: cubic-spline (C2) translation and SLERP orientation.
/// Translation waypoint velocities are solved with
/// joint_trajectory_controller::fill_cubic_spline_velocities; orientation is interpolated directly
/// from the waypoint quaternions.
class CartesianTrajectory
{
public:
  /// times must be strictly increasing; the orientation waypoints are sign-aligned internally.
  CartesianTrajectory(
    const std::vector<double> & times, const std::vector<Eigen::Vector3d> & positions,
    const std::vector<Eigen::Quaterniond> & orientations);

  /// Sample the pose at time t (clamped to [front, back]). Returns false if the path has no
  /// waypoints.
  bool sample(double t, Eigen::Vector3d & position, Eigen::Quaterniond & orientation) const;

  double duration() const;

private:
  /// Cubic-Hermite translation + SLERP orientation within the segment starting at index.
  void interpolate_segment(
    std::size_t index, double time_into_segment, double segment_duration,
    Eigen::Vector3d & position, Eigen::Quaterniond & orientation) const;

  std::vector<double> times_;
  std::vector<Eigen::Vector3d> positions_;
  std::vector<Eigen::Vector3d> velocities_;
  std::vector<Eigen::Quaterniond> orientations_;
};

}  // namespace cartesian_trajectory_controller

#endif  // CARTESIAN_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_HPP_
