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
 * Author: Subhas Das, Denis Stogl
 */

#ifndef TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_
#define TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "force_torque_sensor_controller/force_torque_sensor_controller.hpp"

// subclassing and friending so we can access member varibles
class FriendForceTorqueSensorController : public force_torque_sensor_controller::
  ForceTorqueSensorController
{
  FRIEND_TEST(ForceTorqueSensorControllerTest, SensorNameParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorControllerTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorControllerTest, FrameIdParameterNotSet);
  FRIEND_TEST(ForceTorqueSensorControllerTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(ForceTorqueSensorControllerTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(ForceTorqueSensorControllerTest, ActivateSuccess);
  FRIEND_TEST(ForceTorqueSensorControllerTest, UpdateTest);
  FRIEND_TEST(ForceTorqueSensorControllerTest, SensorStatePublishTest);
};

class ForceTorqueSensorControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpStateController();

protected:
  std::unique_ptr<FriendForceTorqueSensorController> state_controller_;
};

#endif  // TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_
