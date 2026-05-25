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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/test_utils.hpp"
#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using controller_interface::deactivate_succeeds;

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
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorStatePublishTest_with_rotation_offset);
};

class IMUSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpIMUBroadcaster(const std::vector<rclcpp::Parameter> & parameters = {});

protected:
  const std::string sensor_name_ = "imu_sensor";
  const std::string frame_id_ = "imu_sensor_frame";
  const std::array<double, 10> sensor_values_ = {
    {0.1826, 0.3651, 0.5477, 0.7303, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10}};
  hardware_interface::StateInterface::SharedPtr imu_orientation_x_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "orientation.x", "double", "0.1826");
  hardware_interface::StateInterface::SharedPtr imu_orientation_y_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "orientation.y", "double", "0.3651");
  hardware_interface::StateInterface::SharedPtr imu_orientation_z_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "orientation.z", "double", "0.5477");
  hardware_interface::StateInterface::SharedPtr imu_orientation_w_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "orientation.w", "double", "0.7303");
  hardware_interface::StateInterface::SharedPtr imu_angular_velocity_x_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "angular_velocity.x", "double", "5.5");
  hardware_interface::StateInterface::SharedPtr imu_angular_velocity_y_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "angular_velocity.y", "double", "6.6");
  hardware_interface::StateInterface::SharedPtr imu_angular_velocity_z_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "angular_velocity.z", "double", "7.7");
  hardware_interface::StateInterface::SharedPtr imu_linear_acceleration_x_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "linear_acceleration.x", "double", "8.8");
  hardware_interface::StateInterface::SharedPtr imu_linear_acceleration_y_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "linear_acceleration.y", "double", "9.9");
  hardware_interface::StateInterface::SharedPtr imu_linear_acceleration_z_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "linear_acceleration.z", "double", "10.1");

  std::unique_ptr<FriendIMUSensorBroadcaster> imu_broadcaster_;

  void subscribe_and_get_message(sensor_msgs::msg::Imu & imu_msg);
};

#endif  // TEST_IMU_SENSOR_BROADCASTER_HPP_
