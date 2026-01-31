// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mecanum_drive_controller
{
Odometry::Odometry()
: timestamp_(0.0),
  position_x_in_base_frame_(0.0),
  position_y_in_base_frame_(0.0),
  orientation_z_in_base_frame_(0.0),
  velocity_in_base_frame_linear_x(0.0),
  velocity_in_base_frame_linear_y(0.0),
  velocity_in_base_frame_angular_z(0.0),
  sum_of_robot_center_projection_on_X_Y_axis_(0.0),
  wheels_radius_(0.0)
{
}

void Odometry::init(
  const rclcpp::Time & time, std::array<double, PLANAR_POINT_DIM> base_frame_offset)
{
  // Reset timestamp:
  timestamp_ = time;

  // Base frame offset (wrt to center frame).
  base_frame_offset_[0] = base_frame_offset[0];
  base_frame_offset_[1] = base_frame_offset[1];
  base_frame_offset_[2] = base_frame_offset[2];
}

bool Odometry::update(
  const double wheel_front_left_vel, const double wheel_rear_left_vel,
  const double wheel_rear_right_vel, const double wheel_front_right_vel, const double dt)
{
  /// We cannot estimate the speed with very small time intervals:
  // const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001) return false;  // Interval too small to integrate with

  /// Compute FK (i.e. compute mobile robot's body twist out of its wheels velocities):
  /// NOTE: the mecanum IK gives the body speed at the center frame, we then offset this velocity
  ///       at the base frame.
  /// NOTE: in the diff drive the velocity is filtered out, but we prefer to return it raw and
  ///       let the user perform post-processing at will.
  ///       We prefer this way of doing as filtering introduces delay (which makes it difficult
  ///       to interpret and compare behavior curves).

  /// \note The variables meaning:
  /// angular_transformation_from_center_2_base: Rotation transformation matrix, to transform
  /// from center frame to base frame
  /// linear_transformation_from_center_2_base: offset/linear transformation matrix,
  /// to transform from center frame to base frame

  double velocity_in_center_frame_linear_x =
    0.25 * wheels_radius_ *
    (wheel_front_left_vel + wheel_rear_left_vel + wheel_rear_right_vel + wheel_front_right_vel);
  double velocity_in_center_frame_linear_y =
    0.25 * wheels_radius_ *
    (-wheel_front_left_vel + wheel_rear_left_vel - wheel_rear_right_vel + wheel_front_right_vel);
  double velocity_in_center_frame_angular_z =
    0.25 * wheels_radius_ / sum_of_robot_center_projection_on_X_Y_axis_ *
    (-wheel_front_left_vel - wheel_rear_left_vel + wheel_rear_right_vel + wheel_front_right_vel);

  tf2::Quaternion orientation_R_c_b;
  orientation_R_c_b.setRPY(0.0, 0.0, -base_frame_offset_[2]);

  tf2::Matrix3x3 angular_transformation_from_center_2_base = tf2::Matrix3x3((orientation_R_c_b));
  tf2::Vector3 velocity_in_center_frame_w_r_t_base_frame_ =
    angular_transformation_from_center_2_base *
    tf2::Vector3(velocity_in_center_frame_linear_x, velocity_in_center_frame_linear_y, 0.0);
  tf2::Vector3 linear_transformation_from_center_2_base =
    angular_transformation_from_center_2_base *
    tf2::Vector3(-base_frame_offset_[0], -base_frame_offset_[1], 0.0);

  velocity_in_base_frame_linear_x =
    velocity_in_center_frame_w_r_t_base_frame_.x() +
    linear_transformation_from_center_2_base.y() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_linear_y =
    velocity_in_center_frame_w_r_t_base_frame_.y() -
    linear_transformation_from_center_2_base.x() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_angular_z = velocity_in_center_frame_angular_z;

  /// Integration.
  /// NOTE: the position is expressed in the odometry frame , unlike the twist which is
  ///       expressed in the body frame.
  orientation_z_in_base_frame_ += velocity_in_base_frame_angular_z * dt;

  tf2::Quaternion orientation_R_b_odom;
  orientation_R_b_odom.setRPY(0.0, 0.0, orientation_z_in_base_frame_);

  tf2::Matrix3x3 angular_transformation_from_base_2_odom = tf2::Matrix3x3((orientation_R_b_odom));
  tf2::Vector3 velocity_in_base_frame_w_r_t_odom_frame_ =
    angular_transformation_from_base_2_odom *
    tf2::Vector3(velocity_in_base_frame_linear_x, velocity_in_base_frame_linear_y, 0.0);

  position_x_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.x() * dt;
  position_y_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.y() * dt;

  return true;
}

void Odometry::setWheelsParams(
  const double sum_of_robot_center_projection_on_X_Y_axis, const double wheels_radius)
{
  sum_of_robot_center_projection_on_X_Y_axis_ = sum_of_robot_center_projection_on_X_Y_axis;
  wheels_radius_ = wheels_radius;
}
void Odometry::setOdometry(double x, double y, double heading)
{
  position_x_in_base_frame_ = x;
  position_y_in_base_frame_ = y;
  orientation_z_in_base_frame_ = heading;
}
void Odometry::resetOdometry()
{
  position_x_in_base_frame_ = 0.0;
  position_y_in_base_frame_ = 0.0;
  orientation_z_in_base_frame_ = 0.0;
}

}  // namespace mecanum_drive_controller
