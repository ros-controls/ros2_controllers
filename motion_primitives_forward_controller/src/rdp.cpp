// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#include "motion_primitives_forward_controller/rdp.hpp"
#include <cmath>
#include <cstddef>

namespace rdp
{

// Computes the dot product of two vectors
double dot(const Point & a, const Point & b)
{
  double result = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) result += a[i] * b[i];
  return result;
}

// Computes the Euclidean norm (length) of a vector
double norm(const Point & v) { return std::sqrt(dot(v, v)); }

// Subtracts two vectors
Point operator-(const Point & a, const Point & b)
{
  Point result(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) result[i] = a[i] - b[i];
  return result;
}

// Adds two vectors
Point operator+(const Point & a, const Point & b)
{
  Point result(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) result[i] = a[i] + b[i];
  return result;
}

// Multiplies a vector by a scalar
Point operator*(double scalar, const Point & v)
{
  Point result(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) result[i] = scalar * v[i];
  return result;
}

// Computes the perpendicular distance from a point to a line in N-dimensional space
double pointLineDistanceND(const Point & point, const Point & start, const Point & end)
{
  Point lineVec = end - start;
  Point pointVec = point - start;
  double len = norm(lineVec);

  if (len == 0.0) return norm(point - start);

  double projection = dot(pointVec, lineVec) / len;
  Point closest = start + (projection / len) * lineVec;
  return norm(point - closest);
}

// Recursive implementation of the Ramer-Douglas-Peucker algorithm
std::pair<PointList, std::vector<std::size_t>> rdpRecursive(
  const PointList & points, double epsilon, std::size_t offset)
{
  if (points.size() < 2)
  {
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
      indices.push_back(offset + i);
    }
    return {points, indices};
  }

  double dmax = 0.0;
  std::size_t index = 0;

  // Find the point with the maximum distance from the line segment
  for (std::size_t i = 1; i < points.size() - 1; ++i)
  {
    double d = pointLineDistanceND(points[i], points.front(), points.back());
    if (d > dmax)
    {
      index = i;
      dmax = d;
    }
  }

  // If the max distance is greater than epsilon, recursively simplify
  if (dmax > epsilon)
  {
    auto rec1 =
      rdpRecursive(PointList(points.begin(), points.begin() + index + 1), epsilon, offset);
    auto rec2 =
      rdpRecursive(PointList(points.begin() + index, points.end()), epsilon, offset + index);

    // Remove duplicate in res1
    rec1.first.pop_back();
    rec1.second.pop_back();

    // Merge
    rec1.first.insert(rec1.first.end(), rec2.first.begin(), rec2.first.end());
    rec1.second.insert(rec1.second.end(), rec2.second.begin(), rec2.second.end());
    return rec1;
  }
  else
  {
    return {{points.front(), points.back()}, {offset, offset + points.size() - 1}};
  }
}

// Compute angular difference (in radians) between two quaternions
double quaternionAngularDistance(
  const geometry_msgs::msg::Quaternion & q1, const geometry_msgs::msg::Quaternion & q2)
{
  tf2::Quaternion tf_q1, tf_q2;
  tf2::fromMsg(q1, tf_q1);
  tf2::fromMsg(q2, tf_q2);

  tf_q1.normalize();
  tf_q2.normalize();

  double dot = tf_q1.dot(tf_q2);
  dot = std::clamp(dot, -1.0, 1.0);  // numerical safety

  return 2.0 * std::acos(std::abs(dot));  // shortest angle between unit quaternions
}

// Recursively apply the Ramer-Douglas-Peucker algorithm to a list of quaternions.
// Returns a simplified list of quaternions and their corresponding original indices.
std::pair<std::vector<geometry_msgs::msg::Quaternion>, std::vector<size_t>> rdpRecursiveQuaternion(
  const std::vector<geometry_msgs::msg::Quaternion> & quaternions, double epsilon_angle_rad,
  size_t offset)
{
  if (quaternions.size() < 2)
  {
    std::vector<size_t> indices;
    for (size_t i = 0; i < quaternions.size(); ++i) indices.push_back(offset + i);
    return {quaternions, indices};
  }

  double max_angle = 0.0;
  size_t max_index = 0;

  const geometry_msgs::msg::Quaternion & q_start = quaternions.front();
  const geometry_msgs::msg::Quaternion & q_end = quaternions.back();

  // Check intermediate quaternions for deviation from SLERP curve
  for (size_t i = 1; i < quaternions.size() - 1; ++i)
  {
    double t = static_cast<double>(i) / static_cast<double>(quaternions.size() - 1);

    // Interpolate on the SLERP curve between q_start and q_end
    tf2::Quaternion tf_q_start, tf_q_end;
    tf2::fromMsg(q_start, tf_q_start);
    tf2::fromMsg(q_end, tf_q_end);

    tf_q_start.normalize();
    tf_q_end.normalize();

    tf2::Quaternion tf_q_interp = tf_q_start.slerp(tf_q_end, t);
    tf_q_interp.normalize();
    geometry_msgs::msg::Quaternion q_interp = tf2::toMsg(tf_q_interp);

    // Calculate angular distance to actual intermediate quaternion
    double angle_diff = quaternionAngularDistance(q_interp, quaternions[i]);

    if (angle_diff > max_angle)
    {
      max_angle = angle_diff;
      max_index = i;
    }
  }

  if (max_angle > epsilon_angle_rad)
  {
    auto left = rdpRecursiveQuaternion(
      std::vector<geometry_msgs::msg::Quaternion>(
        quaternions.begin(), quaternions.begin() + max_index + 1),
      epsilon_angle_rad, offset);

    auto right = rdpRecursiveQuaternion(
      std::vector<geometry_msgs::msg::Quaternion>(
        quaternions.begin() + max_index, quaternions.end()),
      epsilon_angle_rad, offset + max_index);

    // Avoid duplicate point
    left.first.pop_back();
    left.second.pop_back();

    // Merge results
    left.first.insert(left.first.end(), right.first.begin(), right.first.end());
    left.second.insert(left.second.end(), right.second.begin(), right.second.end());
    return left;
  }
  else
  {
    return {{quaternions.front(), quaternions.back()}, {offset, offset + quaternions.size() - 1}};
  }
}

}  // namespace rdp
