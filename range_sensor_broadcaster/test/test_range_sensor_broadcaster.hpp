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

#ifndef TEST_RANGE_SENSOR_BROADCASTER_HPP_
#define TEST_RANGE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "range_sensor_broadcaster/range_sensor_broadcaster.hpp"

// subclassing and friending so we can access member variables
class FriendRangeSensorBroadcaster : public range_sensor_broadcaster::RangeSensorBroadcaster
{
  FRIEND_TEST(RangeSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(RangeSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(RangeSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(RangeSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(RangeSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(RangeSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(RangeSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(RangeSensorBroadcasterTest, SensorStatePublishTest);
};

class RangeSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpRangeBroadcaster();

protected:
  const std::string sensor_name_ = "range_sensor";
  const std::string frame_id_ = "range_sensor_frame";
  std::array<double, 5> sensor_values_ = {0.0, 3.14, -0.2618, 0.2618, 1.0};

  hardware_interface::StateInterface range_radiation_type_{sensor_name_, "radiation_type", &sensor_values_[0]};
  hardware_interface::StateInterface range_field_of_view_{sensor_name_, "field_of_view", &sensor_values_[1]};
  hardware_interface::StateInterface range_min_range_{sensor_name_, "min_range", &sensor_values_[2]};
  hardware_interface::StateInterface range_max_range_{sensor_name_, "max_range", &sensor_values_[3]};
  hardware_interface::StateInterface range_range_{sensor_name_, "range", &sensor_values_[5]};

  std::unique_ptr<FriendRangeSensorBroadcaster> range_broadcaster_;

  void subscribe_and_get_message(sensor_msgs::msg::Range & range_msg);
};

#endif  // TEST_RANGE_SENSOR_BROADCASTER_HPP_
