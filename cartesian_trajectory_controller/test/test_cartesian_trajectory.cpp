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

#include <gmock/gmock.h>

#include <vector>

#include "cartesian_trajectory_controller/cartesian_trajectory.hpp"

using cartesian_trajectory_controller::CartesianTrajectory;

namespace
{
CartesianTrajectory make_trajectory()
{
  const std::vector<double> times = {0.0, 1.0, 2.0};
  const std::vector<Eigen::Vector3d> positions = {
    {0.0, 0.0, 0.0}, {0.5, 0.1, 0.0}, {1.0, 0.0, 0.2}};
  std::vector<Eigen::Quaterniond> orientations = {
    Eigen::Quaterniond::Identity(),
    Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())),
    Eigen::Quaterniond(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()))};
  return CartesianTrajectory(times, positions, orientations);
}
}  // namespace

// Cubic Hermite interpolates the waypoints, so sampling at a waypoint time returns that waypoint's
// pose.
TEST(TestCartesianTrajectory, passes_through_waypoints)
{
  auto traj = make_trajectory();
  Eigen::Vector3d p;
  Eigen::Quaterniond q;

  ASSERT_TRUE(traj.sample(1.0, p, q));
  EXPECT_NEAR(p.x(), 0.5, 1e-9);
  EXPECT_NEAR(p.y(), 0.1, 1e-9);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.angularDistance(expected), 0.0, 1e-9);
}

// Orientation is time-parameterized SLERP between the bracketing waypoint quaternions.
TEST(TestCartesianTrajectory, orientation_is_slerp)
{
  auto traj = make_trajectory();
  Eigen::Vector3d p;
  Eigen::Quaterniond q;

  ASSERT_TRUE(traj.sample(0.5, p, q));  // midpoint of first segment -> s = 0.5
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q1(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.angularDistance(q0.slerp(0.5, q1)), 0.0, 1e-9);
}

// Sampling outside the span clamps to the endpoints.
TEST(TestCartesianTrajectory, clamps_outside_span)
{
  auto traj = make_trajectory();
  Eigen::Vector3d p;
  Eigen::Quaterniond q;

  ASSERT_TRUE(traj.sample(-1.0, p, q));
  EXPECT_NEAR(p.norm(), 0.0, 1e-9);

  ASSERT_TRUE(traj.sample(5.0, p, q));
  EXPECT_NEAR(p.x(), 1.0, 1e-9);
  EXPECT_NEAR(p.z(), 0.2, 1e-9);
}

// align_quaternions_shortest_arc flips signs so consecutive waypoints share a hemisphere (dot >=
// 0).
TEST(TestCartesianTrajectory, aligns_to_shortest_arc)
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  std::vector<Eigen::Quaterniond> quats = {q, Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z())};
  ASSERT_LT(quats[0].dot(quats[1]), 0.0);  // second waypoint starts on the opposite hemisphere

  cartesian_trajectory_controller::align_quaternions_shortest_arc(quats);
  EXPECT_GE(quats[0].dot(quats[1]), 0.0);  // now on the same hemisphere -> shortest arc
}

// min_segment_duration returns the larger of the linear/angular-limited times, floored at
// min_duration.
TEST(TestCartesianTrajectory, segment_duration_respects_speed_limits)
{
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

  // 0.2 m at 0.1 m/s = 2.0 s; the (zero) rotation does not dominate.
  EXPECT_NEAR(
    cartesian_trajectory_controller::min_segment_duration(
      Eigen::Vector3d::Zero(), identity, Eigen::Vector3d(0.2, 0.0, 0.0), identity, 0.1, 0.5, 0.01),
    2.0, 1e-9);

  // Coincident, unrotated poses -> floored at min_duration.
  EXPECT_NEAR(
    cartesian_trajectory_controller::min_segment_duration(
      Eigen::Vector3d::Zero(), identity, Eigen::Vector3d::Zero(), identity, 0.1, 0.5, 0.01),
    0.01, 1e-9);
}

// A single-waypoint path has zero duration and returns that pose for any query time.
TEST(TestCartesianTrajectory, single_waypoint_clamps)
{
  const std::vector<double> times = {0.5};
  const std::vector<Eigen::Vector3d> positions = {{1.0, 2.0, 3.0}};
  const std::vector<Eigen::Quaterniond> orientations = {
    Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()))};
  CartesianTrajectory traj(times, positions, orientations);

  EXPECT_NEAR(traj.duration(), 0.0, 1e-12);

  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  for (double t : {-1.0, 0.5, 5.0})
  {
    ASSERT_TRUE(traj.sample(t, p, q));
    EXPECT_NEAR((p - positions[0]).norm(), 0.0, 1e-12);
    EXPECT_NEAR(q.angularDistance(orientations[0]), 0.0, 1e-12);
  }
}

// An empty path cannot be sampled.
TEST(TestCartesianTrajectory, empty_trajectory_sample_returns_false)
{
  const std::vector<double> times;
  const std::vector<Eigen::Vector3d> positions;
  const std::vector<Eigen::Quaterniond> orientations;
  CartesianTrajectory traj(times, positions, orientations);

  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  EXPECT_FALSE(traj.sample(0.0, p, q));
}

// Cubic-Hermite translation is C1 across a knot (the velocity is continuous, not a staircase).
TEST(TestCartesianTrajectory, translation_is_smooth_not_staircase)
{
  auto traj = make_trajectory();  // knots at t = 0, 1, 2
  const double eps = 1e-4;
  Eigen::Vector3d p_before, p_at, p_after;
  Eigen::Quaterniond q;
  ASSERT_TRUE(traj.sample(1.0 - eps, p_before, q));
  ASSERT_TRUE(traj.sample(1.0, p_at, q));
  ASSERT_TRUE(traj.sample(1.0 + eps, p_after, q));

  const Eigen::Vector3d v_left = (p_at - p_before) / eps;
  const Eigen::Vector3d v_right = (p_after - p_at) / eps;
  EXPECT_NEAR((v_left - v_right).norm(), 0.0, 1e-2);  // continuous velocity at the knot
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
