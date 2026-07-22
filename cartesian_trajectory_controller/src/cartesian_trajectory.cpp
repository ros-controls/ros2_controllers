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

#include "cartesian_trajectory_controller/cartesian_trajectory.hpp"

#include <algorithm>
#include <cassert>

#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace cartesian_trajectory_controller
{

void align_quaternions_shortest_arc(std::vector<Eigen::Quaterniond> & orientations)
{
  for (size_t i = 1; i < orientations.size(); ++i)
  {
    if (orientations[i - 1].dot(orientations[i]) < 0.0)
    {
      orientations[i].coeffs() *= -1.0;
    }
  }
}

double min_segment_duration(
  const Eigen::Vector3d & from_position, const Eigen::Quaterniond & from_orientation,
  const Eigen::Vector3d & to_position, const Eigen::Quaterniond & to_orientation,
  double max_linear_speed, double max_angular_speed, double min_duration)
{
  const double linear_time = (to_position - from_position).norm() / max_linear_speed;
  const double angular_time = from_orientation.angularDistance(to_orientation) / max_angular_speed;
  return std::max({linear_time, angular_time, min_duration});
}

CartesianTrajectory::CartesianTrajectory(
  const std::vector<double> & times, const std::vector<Eigen::Vector3d> & positions,
  const std::vector<Eigen::Quaterniond> & orientations)
: times_(times), positions_(positions), orientations_(orientations)
{
  assert(times_.size() == positions_.size() && times_.size() == orientations_.size());
  align_quaternions_shortest_arc(orientations_);
  velocities_.assign(positions_.size(), Eigen::Vector3d::Zero());
  if (positions_.size() < 2)
  {
    return;
  }

  // Solve C2 waypoint velocities for x, y, z with the JTC spline helper (rest boundary conditions).
  trajectory_msgs::msg::JointTrajectory xyz;
  xyz.points.resize(positions_.size());
  for (size_t i = 0; i < positions_.size(); ++i)
  {
    xyz.points[i].positions = {positions_[i].x(), positions_[i].y(), positions_[i].z()};
    xyz.points[i].time_from_start = rclcpp::Duration::from_seconds(times_[i]);
  }
  joint_trajectory_controller::fill_cubic_spline_velocities(xyz);
  for (size_t i = 0; i < positions_.size(); ++i)
  {
    const auto & v = xyz.points[i].velocities;
    velocities_[i] = Eigen::Vector3d(v[0], v[1], v[2]);
  }
}

bool CartesianTrajectory::sample(
  double t, Eigen::Vector3d & position, Eigen::Quaterniond & orientation) const
{
  const size_t n = times_.size();
  if (n == 0)
  {
    return false;
  }
  if (t <= times_.front())
  {
    position = positions_.front();
    orientation = orientations_.front();
    return true;
  }
  if (t >= times_.back())
  {
    position = positions_.back();
    orientation = orientations_.back();
    return true;
  }

  size_t i = 0;
  while (i + 1 < n && times_[i + 1] <= t)
  {
    ++i;
  }

  const double h = times_[i + 1] - times_[i];
  if (h <= 0.0)
  {
    position = positions_[i + 1];
    orientation = orientations_[i + 1];
    return true;
  }
  const double u = t - times_[i];
  const double s = u / h;

  // Cubic Hermite per axis, matching the has_velocity branch of Trajectory::interpolate_between_points.
  for (int axis = 0; axis < 3; ++axis)
  {
    const double p0 = positions_[i][axis];
    const double p1 = positions_[i + 1][axis];
    const double v0 = velocities_[i][axis];
    const double v1 = velocities_[i + 1][axis];
    const double c2 = (-3.0 * p0 + 3.0 * p1 - 2.0 * v0 * h - v1 * h) / (h * h);
    const double c3 = (2.0 * p0 - 2.0 * p1 + v0 * h + v1 * h) / (h * h * h);
    position[axis] = p0 + v0 * u + c2 * u * u + c3 * u * u * u;
  }

  orientation = orientations_[i].slerp(s, orientations_[i + 1]);
  return true;
}

double CartesianTrajectory::duration() const
{
  return times_.size() < 2 ? 0.0 : times_.back() - times_.front();
}

}  // namespace cartesian_trajectory_controller
