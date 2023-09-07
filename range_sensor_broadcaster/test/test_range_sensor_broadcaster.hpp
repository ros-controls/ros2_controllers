// Copyright 2023 flochre
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
 * Authors: flochre
 */

#ifndef TEST_RANGE_SENSOR_BROADCASTER_HPP_
#define TEST_RANGE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "range_sensor_broadcaster/range_sensor_broadcaster.hpp"

class RangeSensorBroadcasterTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  // for the sake of the test
  // defining the parameter names same as in test/range_sensor_broadcaster_params.yaml
  const std::string sensor_name_ = "range_sensor";
  const std::string frame_id_ = "range_sensor_frame";

  const double field_of_view_ = 0.1;
  const int radiation_type_ = 1;
  const double min_range_ = 0.1;
  const double max_range_ = 7.0;
  const double variance_ = 1.0;

  double sensor_range_ = 3.1;
  hardware_interface::StateInterface range_{sensor_name_, "range", &sensor_range_};

  std::unique_ptr<range_sensor_broadcaster::RangeSensorBroadcaster> range_broadcaster_;

  controller_interface::return_type init_broadcaster(std::string broadcaster_name);
  controller_interface::CallbackReturn configure_broadcaster(
    std::vector<rclcpp::Parameter> & parameters);
  void subscribe_and_get_message(sensor_msgs::msg::Range & range_msg);
};

#endif  // TEST_RANGE_SENSOR_BROADCASTER_HPP_
