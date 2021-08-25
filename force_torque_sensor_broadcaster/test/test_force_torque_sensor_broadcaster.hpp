// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
 * Authors: Subhas Das, Denis Stogl
 */

#ifndef TEST_FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
#define TEST_FORCE_TORQUE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "force_torque_sensor_broadcaster/force_torque_sensor_broadcaster.hpp"

// subclassing and friending so we can access member variables
class FriendForceTorqueSensorBroadcaster
: public force_torque_sensor_broadcaster::ForceTorqueSensorBroadcaster
{
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(ForceTorqueSensorBroadcasterTest, SensorStatePublishTest);
};

class ForceTorqueSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpFTSBroadcaster();

protected:
  const std::string sensor_name_ = "fts_sensor";
  const std::string frame_id_ = "fts_sensor_frame";
  std::array<double, 6> sensor_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};

  hardware_interface::StateInterface fts_force_x_{sensor_name_, "force.x", &sensor_values_[0]};
  hardware_interface::StateInterface fts_force_y_{sensor_name_, "force.y", &sensor_values_[1]};
  hardware_interface::StateInterface fts_force_z_{sensor_name_, "force.z", &sensor_values_[2]};
  hardware_interface::StateInterface fts_torque_x_{sensor_name_, "torque.x", &sensor_values_[3]};
  hardware_interface::StateInterface fts_torque_y_{sensor_name_, "torque.y", &sensor_values_[4]};
  hardware_interface::StateInterface fts_torque_z_{sensor_name_, "torque.z", &sensor_values_[5]};

  std::unique_ptr<FriendForceTorqueSensorBroadcaster> fts_broadcaster_;

  void subscribe_and_get_message(geometry_msgs::msg::WrenchStamped & wrench_msg);
};

#endif  // TEST_FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
