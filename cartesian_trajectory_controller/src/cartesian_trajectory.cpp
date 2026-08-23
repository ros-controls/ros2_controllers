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

// TODO(vedh1234): implement an exact motion profile instead of this factor.
constexpr double peak_speed_ratio = 1.5;

double min_segment_duration(
  const Eigen::Vector3d & from_position, const Eigen::Quaterniond & from_orientation,
  const Eigen::Vector3d & to_position, const Eigen::Quaterniond & to_orientation,
  double max_linear_speed, double max_angular_speed, double min_duration)
{
  const double linear_time =
    peak_speed_ratio * (to_position - from_position).norm() / max_linear_speed;
  const double angular_time = from_orientation.angularDistance(to_orientation) / max_angular_speed;
  return std::max({linear_time, angular_time, min_duration});
}

CartesianTrajectory::CartesianTrajectory(
  const std::vector<double> & times, const std::vector<Eigen::Vector3d> & positions,
  const std::vector<Eigen::Quaterniond> & orientations, const Eigen::Vector3d & initial_velocity,
  double initial_angular_speed)
: times_(times), positions_(positions), orientations_(orientations)
{
  assert(times_.size() == positions_.size() && times_.size() == orientations_.size());
  align_quaternions_shortest_arc(orientations_);
  velocities_.assign(positions_.size(), Eigen::Vector3d::Zero());
  angles_.assign(positions_.size(), 0.0);
  angle_velocities_.assign(positions_.size(), 0.0);
  if (positions_.size() < 2)
  {
    return;
  }

  // rotation as a scalar channel, monotonically increasing
  for (size_t i = 1; i < orientations_.size(); ++i)
  {
    angles_[i] = angles_[i - 1] + orientations_[i - 1].angularDistance(orientations_[i]);
  }

  // Solve C2 waypoint velocities for x, y, z with the JTC spline helper (rest boundary conditions).
  trajectory_msgs::msg::JointTrajectory channels;
  channels.points.resize(positions_.size());
  for (size_t i = 0; i < positions_.size(); ++i)
  {
    channels.points[i].positions = {
      positions_[i].x(), positions_[i].y(), positions_[i].z(), angles_[i]};
    channels.points[i].time_from_start = rclcpp::Duration::from_seconds(times_[i]);
  }
  // Solving with the start velocity, rather than overwriting it afterwards
  const std::vector<double> start_velocity = {
    initial_velocity.x(), initial_velocity.y(), initial_velocity.z(), initial_angular_speed};
  if (!joint_trajectory_controller::fill_cubic_spline_velocities(channels, start_velocity))
  {
    return;
  }
  for (size_t i = 0; i < positions_.size(); ++i)
  {
    const auto & v = channels.points[i].velocities;
    velocities_[i] = Eigen::Vector3d(v[0], v[1], v[2]);
    angle_velocities_[i] = v[3];
  }
}

bool CartesianTrajectory::sample(
  double t, Eigen::Vector3d & position, Eigen::Quaterniond & orientation) const
{
  const size_t num_waypoints = times_.size();
  if (num_waypoints == 0)
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

  size_t index = 0;
  while (index + 1 < num_waypoints && times_[index + 1] <= t)
  {
    ++index;
  }

  const double segment_duration = times_[index + 1] - times_[index];
  if (segment_duration <= 0.0)
  {
    position = positions_[index + 1];
    orientation = orientations_[index + 1];
    return true;
  }
  interpolate_segment(index, t - times_[index], segment_duration, position, orientation);
  return true;
}

void CartesianTrajectory::interpolate_segment(
  std::size_t index, double time_into_segment, double segment_duration, Eigen::Vector3d & position,
  Eigen::Quaterniond & orientation) const
{
  auto generate_powers = [](int n, double x, double * powers)
  {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  double t_powers[4];
  double duration_powers[4];
  generate_powers(3, time_into_segment, t_powers);
  generate_powers(3, segment_duration, duration_powers);

  auto hermite = [&](double start_pos, double start_vel, double end_pos, double end_vel)
  {
    double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * duration_powers[1] -
                       end_vel * duration_powers[1]) /
                      duration_powers[2];
    coefficients[3] = (2.0 * start_pos - 2.0 * end_pos + start_vel * duration_powers[1] +
                       end_vel * duration_powers[1]) /
                      duration_powers[3];
    return t_powers[0] * coefficients[0] + t_powers[1] * coefficients[1] +
           t_powers[2] * coefficients[2] + t_powers[3] * coefficients[3];
  };

  for (int axis = 0; axis < 3; ++axis)
  {
    position[axis] = hermite(
      positions_[index][axis], velocities_[index][axis], positions_[index + 1][axis],
      velocities_[index + 1][axis]);
  }

  // slerp on the solved angle, so rotation follows the same profile as translation
  const double swept = angles_[index + 1] - angles_[index];
  double fraction = time_into_segment / segment_duration;  // no rotation: parameter is unused
  if (swept > 1e-12)
  {
    const double angle = hermite(
      angles_[index], angle_velocities_[index], angles_[index + 1], angle_velocities_[index + 1]);
    fraction = (angle - angles_[index]) / swept;
  }
  orientation = orientations_[index].slerp(fraction, orientations_[index + 1]);
}

double CartesianTrajectory::duration() const
{
  return times_.size() < 2 ? 0.0 : times_.back() - times_.front();
}

}  // namespace cartesian_trajectory_controller
