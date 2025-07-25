// Copyright 2025 AIT Austrian Institute of Technology GmbH
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

/*
 * Authors: Christoph Froehlich
 * Adapted from
 * https://github.com/ros-perception/imu_pipeline/blob/jazzy/imu_transformer/include/imu_transformer/tf2_sensor_msgs.h
 */

#ifndef IMU_SENSOR_BROADCASTER__IMU_TRANSFORM_HPP_
#define IMU_SENSOR_BROADCASTER__IMU_TRANSFORM_HPP_

#include <Eigen/Dense>
#include "sensor_msgs/msg/imu.hpp"

namespace imu_sensor_broadcaster
{
/**
 * @brief Euler to quaternion Z-Y'-X'' convention
 * https://eigen.tuxfamily.org/dox-devel/group__TutorialGeometry.html
 */
Eigen::Quaterniond quat_from_euler(double roll, double pitch, double yaw)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

/**
 * @brief Transforms a covariance array from one frame to another
 */
inline void transformCovariance(
  const std::array<double, 9> & in, std::array<double, 9> & out, Eigen::Quaterniond r)
{
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());
  cov_out = r * cov_in * r.inverse();
}

/**
 * @brief Transforms sensor_msgs::Imu data from one frame to another
 */
inline void doTransform(
  const sensor_msgs::msg::Imu & imu_in, sensor_msgs::msg::Imu & imu_out, const Eigen::Quaterniond r)
{
  imu_out.header = imu_in.header;

  Eigen::Transform<double, 3, Eigen::Affine> t(r);
  Eigen::Vector3d vel =
    t * Eigen::Vector3d(
          imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

  imu_out.angular_velocity.x = vel.x();
  imu_out.angular_velocity.y = vel.y();
  imu_out.angular_velocity.z = vel.z();

  transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

  Eigen::Vector3d accel =
    t * Eigen::Vector3d(
          imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

  imu_out.linear_acceleration.x = accel.x();
  imu_out.linear_acceleration.y = accel.y();
  imu_out.linear_acceleration.z = accel.z();

  transformCovariance(
    imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

  // Orientation expresses attitude of the new frame_id in a fixed world frame. This is why the
  // transform here applies in the opposite direction.
  Eigen::Quaterniond orientation =
    Eigen::Quaterniond(
      imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) *
    r.inverse();

  imu_out.orientation.w = orientation.w();
  imu_out.orientation.x = orientation.x();
  imu_out.orientation.y = orientation.y();
  imu_out.orientation.z = orientation.z();

  // Orientation is measured relative to the fixed world frame, so it doesn't change when applying a
  // static transform to the sensor frame.
  imu_out.orientation_covariance = imu_in.orientation_covariance;
}
}  // namespace imu_sensor_broadcaster
#endif  // IMU_SENSOR_BROADCASTER__IMU_TRANSFORM_HPP_
