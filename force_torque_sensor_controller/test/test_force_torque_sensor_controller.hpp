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

#ifndef TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_
#define TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "force_torque_sensor_controller/force_torque_sensor_controller.hpp"

// subclassing and friending so we can access member varibles
class FriendForceTorqueSensorController : public force_torque_sensor_controller::ForceTorqueSensorController
{
  FRIEND_TEST(ForceTorqueSensorControllerTest, ConfigureErrorTest);
  FRIEND_TEST(ForceTorqueSensorControllerTest, ConfigureSuccessTest);
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
  // dummy joint state values used for tests
  const std::vector<std::string> sensor_names_ = {"sensor1", "sensor2", "sensor3"};
  std::vector<double> sensor_values_ = {1.1, 2.1, 3.1};

  hardware_interface::StateInterface sensor_1_fx_state_{sensor_names_[0], "fx",
    &sensor_values_[0]};
  hardware_interface::StateInterface sensor_2_fx_state_{sensor_names_[1], "fx",
    &sensor_values_[1]};
  hardware_interface::StateInterface sensor_3_fx_state_{sensor_names_[2], "fx",
    &sensor_values_[2]};
  hardware_interface::StateInterface sensor_1_tz_state_{sensor_names_[0], "tz",
    &sensor_values_[0]};
  hardware_interface::StateInterface sensor_2_tz_state_{sensor_names_[1], "tz",
    &sensor_values_[1]};
  hardware_interface::StateInterface sensor_3_tz_state_{sensor_names_[2], "tz",
    &sensor_values_[2]};

  std::unique_ptr<FriendForceTorqueSensorController> state_controller_;
};

#endif  // TEST_FORCE_TORQUE_SENSOR_CONTROLLER_HPP_
