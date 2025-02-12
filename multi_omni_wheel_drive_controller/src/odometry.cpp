// Copyright 2025 Aarav Gupta
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

#include "multi_omni_wheel_drive_controller/odometry.hpp"

#include <cmath>
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace multi_omni_wheel_drive_controller
{
Odometry::Odometry()
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_vel_(0.0),
  linear_y_vel_(0.0),
  angular_vel_(0.0),
  robot_radius_(0.0),
  wheels_old_pos_(0.0)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset timestamp:
  timestamp_ = time;
}

bool Odometry::update(const std::vector<double> & wheels_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Estimate velocity of wheels using old and current position [m/s]:
  std::vector<double> wheels_vel(wheels_pos.size());
  for (size_t i = 0; i < static_cast<size_t>(wheels_pos.size()); ++i)
  {
    wheels_vel[i] = (wheels_pos[i] - wheels_old_pos_[i]) / dt;
    wheels_old_pos_[i] = wheels_pos[i];
  }

  updateFromVelocity(wheels_vel, time);
  return true;
}

bool Odometry::updateFromVelocity(const std::vector<double> & wheels_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Compute linear and angular velocities of the robot:
  const Eigen::Vector3d robot_velocity = compute_robot_velocity(wheels_vel);

  // Integrate odometry:
  integrate(robot_velocity(0) * dt, robot_velocity(1) * dt, robot_velocity(2) * dt);

  timestamp_ = time;

  linear_x_vel_ = robot_velocity(0);
  linear_y_vel_ = robot_velocity(1);
  angular_vel_ = robot_velocity(2);

  return true;
}

Eigen::Vector3d Odometry::compute_robot_velocity(const std::vector<double> & wheels_vel) const
{
  Eigen::MatrixXd A(wheels_vel.size(), 3);   // Transformation Matrix
  Eigen::VectorXd omega(wheels_vel.size());  // Wheel velocities vector

  const double angle_bw_wheels = (2 * M_PI) / static_cast<double>(wheels_vel.size());

  for (size_t i = 0; i < wheels_vel.size(); ++i)
  {
    // Define the transformation matrix
    const double theta = (angle_bw_wheels * static_cast<double>(i)) + wheel_offset_;
    A(static_cast<int>(i), 0) = std::sin(theta);
    A(static_cast<int>(i), 1) = -std::cos(theta);
    A(static_cast<int>(i), 2) = -robot_radius_;

    // Define the wheel velocities vector
    omega(static_cast<int>(i)) = wheels_vel[i];
  }

  // Compute the robot velocities using SVD decomposition
  const Eigen::Vector3d V = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(omega);

  return V;
}

void Odometry::updateOpenLoop(
  const double & linear_x_vel, const double & linear_y_vel, const double & angular_vel,
  const rclcpp::Time & time)
{
  // Save last linear and angular velocity:
  linear_x_vel_ = linear_x_vel;
  linear_y_vel_ = linear_y_vel;
  angular_vel_ = angular_vel;

  // Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrate(linear_x_vel * dt, linear_y_vel * dt, angular_vel * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setParams(
  const double & robot_radius, const double & wheel_offset, const size_t & wheel_count)
{
  robot_radius_ = robot_radius;
  wheel_offset_ = wheel_offset;
  wheels_old_pos_.resize(wheel_count, 0.0);
}

void Odometry::integrate(const double & dx, const double & dy, const double & dheading)
{
  heading_ = heading_ + dheading;

  // Create a quaternion representing the rotation from the base frame to the odom frame
  tf2::Quaternion q_base_to_odom;
  q_base_to_odom.setRPY(0.0, 0.0, heading_);
  // Create a rotation matrix from the quaternion
  const tf2::Matrix3x3 rot_base_to_odom = tf2::Matrix3x3(q_base_to_odom);

  // Define the displacement vector in the base frame
  const tf2::Vector3 displacement_base(dx, dy, 0.0);
  // Rotate the displacement in the base frame into the odom frame
  const tf2::Vector3 displacement_odom = rot_base_to_odom * displacement_base;

  x_ = x_ + displacement_odom.x();
  y_ = y_ + displacement_odom.y();
}

}  // namespace multi_omni_wheel_drive_controller
