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

#ifndef OMNI_WHEEL_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define OMNI_WHEEL_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <Eigen/Dense>
#include <vector>

#include "rclcpp/time.hpp"

namespace omni_wheel_drive_controller
{
class Odometry
{
public:
  Odometry();

  bool updateFromPos(const std::vector<double> & wheels_pos, const rclcpp::Time & time);
  bool updateFromVel(const std::vector<double> & wheels_vel, const rclcpp::Time & time);
  bool updateOpenLoop(
    const double & linear_x_vel, const double & linear_y_vel, const double & angular_vel,
    const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearXVel() const { return linear_x_vel_; }
  double getLinearYVel() const { return linear_y_vel_; }
  double getAngularVel() const { return angular_vel_; }

  void setParams(
    const double & robot_radius, const double & wheel_radius, const double & wheel_offset,
    const size_t & wheel_count);

private:
  Eigen::Vector3d compute_robot_velocity(const std::vector<double> & wheels_vel) const;
  void integrate(const double & dx, const double & dy, const double & dheading);

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        // [m]
  double y_;        // [m]
  double heading_;  // [rads]

  // Current velocity:
  double linear_x_vel_;  // [m/s]
  double linear_y_vel_;  // [m/s]
  double angular_vel_;   // [rads/s]

  // Robot kinematic parameters:
  double robot_radius_;  // [m]
  double wheel_radius_;  // [m]
  double wheel_offset_;  // [rads]

  // Previous wheel positions/states [rads]:
  std::vector<double> wheels_old_pos_;
};

}  // namespace omni_wheel_drive_controller

#endif  // OMNI_WHEEL_DRIVE_CONTROLLER__ODOMETRY_HPP_
