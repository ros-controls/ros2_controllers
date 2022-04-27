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

#ifndef TEST_CAMERA_SENSOR_BROADCASTER_HPP_
#define TEST_CAMERA_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "camera_sensor_broadcaster/camera_sensor_broadcaster.hpp"

// subclassing and friending so we can access member variables
class FriendCameraSensorBroadcaster : public camera_sensor_broadcaster::CameraSensorBroadcaster
{
  FRIEND_TEST(CameraSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(CameraSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(CameraSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(CameraSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(CameraSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(CameraSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(CameraSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(CameraSensorBroadcasterTest, SensorStatePublishTest);
};

class CameraSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpCameraBroadcaster();

protected:
  const std::string sensor_name_ = "camera_sensor";
  const std::string frame_id_ = "camera_sensor_frame";
  double sensor_values_[2] = {420.0, 960.0};

  hardware_interface::StateInterface camera_height_{sensor_name_, "height", &sensor_values_[0]};
  hardware_interface::StateInterface camera_width_{sensor_name_, "width", &sensor_values_[1]};

  // std::array<double, 10> sensor_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10};
  // hardware_interface::StateInterface camera_orientation_x_{
  //   sensor_name_, "orientation.x", &sensor_values_[0]};
  // hardware_interface::StateInterface camera_orientation_y_{
  //   sensor_name_, "orientation.y", &sensor_values_[1]};
  // hardware_interface::StateInterface camera_orientation_z_{
  //   sensor_name_, "orientation.z", &sensor_values_[2]};
  // hardware_interface::StateInterface camera_orientation_w_{
  //   sensor_name_, "orientation.w", &sensor_values_[3]};
  // hardware_interface::StateInterface camera_angular_velocity_x_{
  //   sensor_name_, "angular_velocity.x", &sensor_values_[4]};
  // hardware_interface::StateInterface camera_angular_velocity_y_{
  //   sensor_name_, "angular_velocity.y", &sensor_values_[5]};
  // hardware_interface::StateInterface camera_angular_velocity_z_{
  //   sensor_name_, "angular_velocity.z", &sensor_values_[6]};
  // hardware_interface::StateInterface camera_linear_acceleration_x_{
  //   sensor_name_, "linear_acceleration.x", &sensor_values_[7]};
  // hardware_interface::StateInterface camera_linear_acceleration_y_{
  //   sensor_name_, "linear_acceleration.y", &sensor_values_[8]};
  // hardware_interface::StateInterface camera_linear_acceleration_z_{
  //   sensor_name_, "linear_acceleration.z", &sensor_values_[9]};

  std::unique_ptr<FriendCameraSensorBroadcaster> camera_broadcaster_;

  void subscribe_and_get_message(sensor_msgs::msg::Image & camera_msg);
};

#endif  // TEST_CAMERA_SENSOR_BROADCASTER_HPP_
