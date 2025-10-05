// Copyright 2020 PAL Robotics SL.
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

#ifndef TEST_CHAINED_FILTER_HPP_
#define TEST_CHAINED_FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "chained_filter_controller/chained_filter.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using hardware_interface::HW_IF_POSITION;
using hardware_interface::StateInterface;

class ChainedFilterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController(
    const std::string node_name = "test_chained_filter",
    const std::vector<rclcpp::Parameter> & parameters = {});

protected:
  std::unique_ptr<chained_filter_controller::ChainedFilter> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"wheel_left"};
  std::vector<double> joint_states_ = {1.1};

  StateInterface::SharedPtr joint_1_pos_ =
    std::make_shared<StateInterface>(joint_names_[0], HW_IF_POSITION, &joint_states_[0]);
  rclcpp::executors::SingleThreadedExecutor executor;
};

#endif  // TEST_CHAINED_FILTER_HPP_
