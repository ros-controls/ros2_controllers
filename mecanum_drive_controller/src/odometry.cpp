// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "mecanum_drive_controller/odometry.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <boost/bind.hpp>
using namespace std;
namespace mecanum_drive_controller
{

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  position_x_base_frame_(0.0),
  position_y_base_frame_(0.0),
  orientation_z_base_frame_(0.0),
  wheels_k_(0.0),
  wheels_radius_(0.0)
{
}

void Odometry::init(const rclcpp::Time & time, double base_frame_offset[PLANAR_POINT_DIM])
{
  // Reset timestamp:
  timestamp_ = time;

  // Base frame offset (wrt to center frame).
  base_frame_offset_[0] = base_frame_offset[0];
  base_frame_offset_[1] = base_frame_offset[1];
  base_frame_offset_[2] = base_frame_offset[2];
}

bool Odometry::update(
  double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const double dt)
{
  /// We cannot estimate the speed with very small time intervals:
  // const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001) return false;  // Interval too small to integrate with

  // timestamp_ = time;

  /// Compute FK (i.e. compute mobile robot's body twist out of its wheels velocities):
  /// NOTE: we use the IK of the mecanum wheels which we invert using a pseudo-inverse.
  /// NOTE: the mecanum IK gives the body speed at the center frame, we then offset this velocity
  ///       at the base frame.
  /// NOTE: in the diff drive the velocity is filtered out, but we prefer to return it raw and
  ///       let the user perform post-processing at will.
  ///       We prefer this way of doing as filtering introduces delay (which makes it difficult
  ///       to interpret and compare behavior curves).

  /// \note The variables meaning:
  /// angular_transformation_from_center_2_base: Rotation transformation matrix, to transform from center frame to base frame
  /// linear_transformation_from_center_2_base: offset/linear transformation matrix, to transform from center frame to base frame

  body_velocity_center_frame_.linear_x = 0.25 * wheels_radius_ * (wheel0_vel + wheel1_vel + wheel2_vel + wheel3_vel);
  body_velocity_center_frame_.linear_y =
    0.25 * wheels_radius_ * (-wheel0_vel + wheel1_vel - wheel2_vel + wheel3_vel);
  body_velocity_center_frame_.angular_z =
    0.25 * wheels_radius_ / wheels_k_ * (-wheel0_vel - wheel1_vel + wheel2_vel + wheel3_vel);

  tf2::Quaternion orientation_R_c_b;
  orientation_R_c_b.setRPY(0.0, 0.0, -base_frame_offset_[2]);

  tf2::Matrix3x3 angular_transformation_from_center_2_base = tf2::Matrix3x3((orientation_R_c_b));
  tf2::Vector3 body_velocity_center_frame_w_r_t_base_frame_ =angular_transformation_from_center_2_base * tf2::Vector3(body_velocity_center_frame_.linear_x, body_velocity_center_frame_.linear_y, 0.0);
  tf2::Vector3 linear_transformation_from_center_2_base =angular_transformation_from_center_2_base * tf2::Vector3(-base_frame_offset_[0], -base_frame_offset_[1], 0.0);

  body_velocity_base_frame_.linear_x = body_velocity_center_frame_w_r_t_base_frame_.x() + linear_transformation_from_center_2_base.y() * body_velocity_center_frame_.angular_z;
  body_velocity_base_frame_.linear_y = body_velocity_center_frame_w_r_t_base_frame_.y() - linear_transformation_from_center_2_base.x() * body_velocity_center_frame_.angular_z;
  body_velocity_base_frame_.angular_z = body_velocity_center_frame_.angular_z;

  /// Integration.
  /// NOTE: the position is expressed in the odometry frame (frame b0), unlike the twist which is
  ///       expressed in the body frame (frame b).
  orientation_z_base_frame_ += body_velocity_base_frame_.angular_z * dt;

  tf2::Quaternion orientation_R_b_odom;
  orientation_R_b_odom.setRPY(0.0, 0.0, -base_frame_offset_[2]);

  tf2::Matrix3x3 angular_transformation_from_base_2_odom = tf2::Matrix3x3((orientation_R_b_odom));
  tf2::Vector3 body_velocity_base_frame_w_r_t_odom_frame_ = angular_transformation_from_base_2_odom * tf2::Vector3(body_velocity_base_frame_.linear_x, body_velocity_base_frame_.linear_y, 0.0);

  position_x_base_frame_ += body_velocity_base_frame_w_r_t_odom_frame_.x() * dt;
  position_y_base_frame_ += body_velocity_base_frame_w_r_t_odom_frame_.y() * dt;
  fprintf(stderr, " position_x_base_frame_ = %f  \n", position_x_base_frame_);
  fprintf(stderr, " position_y_base_frame_ = %f  \n", position_y_base_frame_);

  return true;
}

void Odometry::setWheelsParams(double wheels_k, double wheels_radius)
{
  wheels_k_ = wheels_k;
  wheels_radius_ = wheels_radius;
}

}  // namespace mecanum_drive_controller
