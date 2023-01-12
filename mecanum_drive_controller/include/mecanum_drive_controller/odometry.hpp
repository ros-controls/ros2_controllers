// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
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

#ifndef MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/twist.hpp"


#define PLANAR_POINT_DIM 3

namespace mecanum_drive_controller
{

/// \brief The Odometry class handles odometry readings
/// (2D pose and velocity with related timestamp)
class Odometry
{
public:
  /// Integration function, used to integrate the odometry:
  typedef std::function<void(double, double, double)> IntegrationFunction;

  /// \brief Constructor
  /// Timestamp will get the current time value
  /// Value will be set to zero
  /// \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
  Odometry(size_t velocity_rolling_window_size = 10);

  /// \brief Initialize the odometry
  /// \param time Current time
  void init(const rclcpp::Time & time, std::array<double, PLANAR_POINT_DIM> base_frame_offset);

  /// \brief Updates the odometry class with latest wheels position
  /// \param wheel0_vel  Wheel velocity [rad/s]
  /// \param wheel1_vel  Wheel velocity [rad/s]
  /// \param wheel2_vel  Wheel velocity [rad/s]
  /// \param wheel3_vel  Wheel velocity [rad/s]
  /// \param time      Current time
  /// \return true if the odometry is actually updated
  bool update(
    double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const double dt);

  /// \return position (x component) [m]
  double getX() const { return position_x_in_base_frame_; }
  /// \return position (y component) [m]
  double getY() const { return position_y_in_base_frame_; }
  /// \return orientation (z component) [m]
  double getRz() const { return orientation_z_in_base_frame_; }
  /// \return body velocity of the base frame (linear x component) [m/s]
  double getVx() const { return velocity_in_base_frame_linear_x; }
  /// \return body velocity of the base frame (linear y component) [m/s]
  double getVy() const { return velocity_in_base_frame_linear_y; }
  /// \return body velocity of the base frame (angular z component) [m/s]
  double getWz() const { return velocity_in_base_frame_angular_z;; }

  /// \brief Sets the wheels parameters: mecanum geometric param and radius
  /// \param wheels_k       Wheels geometric param (used in mecanum wheels' ik) [m]
  /// \param wheels_radius  Wheels radius [m]
  void setWheelsParams(double wheels_k, double wheels_radius);

private:

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Reference frame (wrt to center frame).
  std::array<double, PLANAR_POINT_DIM> base_frame_offset_;

  /// Current pose:
  double position_x_in_base_frame_;  // [m]
  double position_y_in_base_frame_;  // [m]
  double orientation_z_in_base_frame_;  // [rad]

  double velocity_in_base_frame_linear_x;  // [m/s]
  double velocity_in_base_frame_linear_y;  // [m/s] 
  double velocity_in_base_frame_angular_z;    // [rad/s]

  /// Wheels kinematic parameters [m]:
  /// lx and ly represent the distance from the robot's center to the wheels projected on 
  /// the x and y axis with origin at robots center respectively, wheels_k_ = lx+ly
  double wheels_k_;
  double wheels_radius_;
};

}  // namespace mecanum_drive_controller

#endif /* MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_ */
