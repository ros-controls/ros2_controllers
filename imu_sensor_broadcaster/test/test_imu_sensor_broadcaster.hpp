// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#ifndef TEST_IMU_SENSOR_BROADCASTER_HPP_
#define TEST_IMU_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"

// subclassing and friending so we can access member variables
class FriendIMUSensorBroadcaster : public imu_sensor_broadcaster::IMUSensorBroadcaster
{
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(IMUSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(IMUSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(IMUSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorStatePublishTest);
};

class IMUSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpIMUBroadcaster();

protected:
  const std::string sensor_name_ = "imu_sensor";
  const std::string frame_id_ = "imu_sensor_frame";
  std::array<double, 10> sensor_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10};
  hardware_interface::StateInterface imu_orientation_x_{
    sensor_name_, "orientation.x", &sensor_values_[0]};
  hardware_interface::StateInterface imu_orientation_y_{
    sensor_name_, "orientation.y", &sensor_values_[1]};
  hardware_interface::StateInterface imu_orientation_z_{
    sensor_name_, "orientation.z", &sensor_values_[2]};
  hardware_interface::StateInterface imu_orientation_w_{
    sensor_name_, "orientation.w", &sensor_values_[3]};
  hardware_interface::StateInterface imu_angular_velocity_x_{
    sensor_name_, "angular_velocity.x", &sensor_values_[4]};
  hardware_interface::StateInterface imu_angular_velocity_y_{
    sensor_name_, "angular_velocity.y", &sensor_values_[5]};
  hardware_interface::StateInterface imu_angular_velocity_z_{
    sensor_name_, "angular_velocity.z", &sensor_values_[6]};
  hardware_interface::StateInterface imu_linear_acceleration_x_{
    sensor_name_, "linear_acceleration.x", &sensor_values_[7]};
  hardware_interface::StateInterface imu_linear_acceleration_y_{
    sensor_name_, "linear_acceleration.y", &sensor_values_[8]};
  hardware_interface::StateInterface imu_linear_acceleration_z_{
    sensor_name_, "linear_acceleration.z", &sensor_values_[9]};

  std::unique_ptr<FriendIMUSensorBroadcaster> imu_broadcaster_;

  void subscribe_and_get_message(sensor_msgs::msg::Imu & imu_msg);
};

#endif  // TEST_IMU_SENSOR_BROADCASTER_HPP_
