// Copyright (c) 2025 AIT Austrian Institute of Technology GmbH
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

// adapted from
// https://github.com/ros-perception/imu_pipeline/blob/jazzy/imu_transformer/test/test_imu_transforms.cpp

#include <array>

#include "Eigen/Dense"
#include "gmock/gmock.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "imu_sensor_broadcaster/imu_transform.hpp"

using imu_sensor_broadcaster::doTransform;
using imu_sensor_broadcaster::quat_from_euler;
using imu_sensor_broadcaster::transformCovariance;

void compareCovariances(const std::array<double, 9> & c1, const std::array<double, 9> & c2)
{
  for (size_t i = 0; i < 9; ++i) EXPECT_NEAR(c1[i], c2[i], 1e-6) << "Wrong value at position " << i;
}

TEST(Covariance, Transform)
{
  std::array<double, 9> in = {{1, 0, 0, 0, 2, 0, 0, 0, 3}};
  std::array<double, 9> expectedOut = {{1, 0, 0, 0, 2, 0, 0, 0, 3}};
  std::array<double, 9> out{};
  Eigen::Quaterniond q(1, 0, 0, 0);
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)));
  expectedOut = {{2, 0, 0, 0, 1, 0, 0, 0, 3}};
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 0, 0)));
  expectedOut = {{1, 0, 0, 0, 3, 0, 0, 0, 2}};
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 1, 0)));
  expectedOut = {{3, 0, 0, 0, 2, 0, 0, 0, 1}};
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1)));
  expectedOut = {{2.5, -0.5, 3, 1, 0, -1, -1.5, 2, 0.5}};
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  expectedOut = {{1.5, -1, 1, 2, 2, -1.5, -0.5, 3, -0.5}};
  transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
}

// check if we use the same euler angle definition as tf2
TEST(Quat, quatFromEuler)
{
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(M_PI, 0., M_PI_2);
  const auto q = quat_from_euler(M_PI, 0., M_PI_2);
  EXPECT_NEAR(tf_quat.x(), q.x(), 1e-6);
  EXPECT_NEAR(tf_quat.y(), q.y(), 1e-6);
  EXPECT_NEAR(tf_quat.z(), q.z(), 1e-6);
  EXPECT_NEAR(tf_quat.w(), q.w(), 1e-6);
}

void prepareImuMsg(sensor_msgs::msg::Imu & msg)
{
  msg.header.frame_id = "test";
  msg.header.stamp.sec = 1;
  msg.angular_velocity.x = 1;
  msg.angular_velocity.y = 2;
  msg.angular_velocity.z = 3;
  msg.angular_velocity_covariance = {{1, 0, 0, 0, 2, 0, 0, 0, 3}};
  msg.linear_acceleration.x = 1;
  msg.linear_acceleration.y = 2;
  msg.linear_acceleration.z = 3;
  msg.linear_acceleration_covariance = {{1, 0, 0, 0, 2, 0, 0, 0, 3}};
  msg.orientation.w = 1;
  msg.orientation_covariance = {{1, 0, 0, 0, 2, 0, 0, 0, 3}};
}

TEST(Imu, DoTransformYaw)
{
  // Q = +90 degrees yaw

  sensor_msgs::msg::Imu msg;
  prepareImuMsg(msg);

  const auto q = quat_from_euler(0, 0, M_PI_2);
  tf2::Quaternion tf_quat(q.x(), q.y(), q.z(), q.w());
  sensor_msgs::msg::Imu out;
  doTransform(msg, out, q);

  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(-msg.angular_velocity.y, out.angular_velocity.x, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.x, out.angular_velocity.y, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.z, out.angular_velocity.z, 1e-6);
  EXPECT_NEAR(-msg.linear_acceleration.y, out.linear_acceleration.x, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.x, out.linear_acceleration.y, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.z, out.linear_acceleration.z, 1e-6);
  // Transforming orientation means expressing the attitude of the new frame in the same world frame
  // (i.e. you have data in imu frame and want to ask what is the world-referenced orientation of
  // the base_link frame that is attached to this IMU). This is why the orientation change goes the
  // other way than the transform.
  tf2::Quaternion rot;
  tf2::convert(out.orientation, rot);
  EXPECT_NEAR(0, rot.angleShortestPath(tf_quat.inverse()), 1e-6);

  compareCovariances({{2, 0, 0, 0, 1, 0, 0, 0, 3}}, out.angular_velocity_covariance);
  compareCovariances({{2, 0, 0, 0, 1, 0, 0, 0, 3}}, out.linear_acceleration_covariance);
  // Orientation covariance stays as it is measured regarding the fixed world frame
  compareCovariances(msg.orientation_covariance, out.orientation_covariance);
}

TEST(Imu, DoTransformEnuNed)
{
  // Q = ENU->NED transform

  sensor_msgs::msg::Imu msg;
  prepareImuMsg(msg);

  const auto q = quat_from_euler(M_PI, 0, M_PI_2);
  tf2::Quaternion tf_quat(q.x(), q.y(), q.z(), q.w());
  sensor_msgs::msg::Imu out;
  doTransform(msg, out, q);

  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(msg.angular_velocity.y, out.angular_velocity.x, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.x, out.angular_velocity.y, 1e-6);
  EXPECT_NEAR(-msg.angular_velocity.z, out.angular_velocity.z, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.y, out.linear_acceleration.x, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.x, out.linear_acceleration.y, 1e-6);
  EXPECT_NEAR(-msg.linear_acceleration.z, out.linear_acceleration.z, 1e-6);
  // Transforming orientation means expressing the attitude of the new frame in the same world frame
  // (i.e. you have data in imu frame and want to ask what is the world-referenced orientation of
  // the base_link frame that is attached to this IMU). This is why the orientation change goes the
  // other way than the transform.
  tf2::Quaternion rot;
  tf2::convert(out.orientation, rot);
  EXPECT_NEAR(0, rot.angleShortestPath(tf_quat.inverse()), 1e-6);

  compareCovariances({{2, 0, 0, 0, 1, 0, 0, 0, 3}}, out.angular_velocity_covariance);
  compareCovariances({{2, 0, 0, 0, 1, 0, 0, 0, 3}}, out.linear_acceleration_covariance);
  // Orientation covariance stays as it is measured regarding the fixed world frame
  compareCovariances(msg.orientation_covariance, out.orientation_covariance);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
