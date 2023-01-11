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
  void init(const rclcpp::Time & time, double base_frame_offset[PLANAR_POINT_DIM]);

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
  double getX() const { return position_x_base_frame_; }
  /// \return position (y component) [m]
  double getY() const { return position_y_base_frame_; }
  /// \return orientation (z component) [m]
  double getRz() const { return orientation_z_base_frame_; }
  /// \return body velocity of the base frame (linear x component) [m/s]
  double getVx() const { return body_velocity_base_frame_.linear_x; }
  /// \return body velocity of the base frame (linear y component) [m/s]
  double getVy() const { return body_velocity_base_frame_.linear_y; }
  /// \return body velocity of the base frame (angular z component) [m/s]
  double getWz() const { return body_velocity_base_frame_.angular_z; }

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
  double position_x_base_frame_;  // [m]
  double position_y_base_frame_;  // [m]
  double orientation_z_base_frame_;  // [rad]

  /// Current velocity.
  /// \note The indices meaning is the following:
  ///    b : base frame
  ///    c : center frame
  ///    body_velocity_base_frame_: body_velocity w.r.t base frame
  ///    body_velocity_center_frame_: body_velocity w.r.t center frame
  ///    Oc: origin of the center frame
  ///    b0: initial position if the base frame
  ///    c0: initial position of the center frame

  struct body_velocity{
    double linear_x;  // [m/s]
    double linear_y;  // [m/s]
    double angular_z;  // [rad/s]

    body_velocity() : linear_x(0.0), linear_y(0.0), angular_z(0.0) {}
  };

  body_velocity body_velocity_base_frame_;
  body_velocity body_velocity_center_frame_;
  // // body_velocity_base_frame_.linear_x;  // [m/s] body_velocity_base_frame_.linear_x
  // // double body_velocity_base_frame_.linear_y;  // [m/s] 
  // // double body_velocity_base_frame_.angular_z;     // [rad/s]

  /// Wheels kinematic parameters [m]:
  double wheels_k_;
  double wheels_radius_;
};

}  // namespace mecanum_drive_controller

#endif /* MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_ */
