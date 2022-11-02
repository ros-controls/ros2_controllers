// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef TEST_GPIO_COMMAND_CONTROLLER_HPP_
#define TEST_GPIO_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "gpio_controllers/gpio_command_controller.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;

// subclassing and friending so we can access member variables
class FriendGpioCommandController : public gpio_controllers::GpioCommandController
{
  FRIEND_TEST(GpioCommandControllerTest, CommandSuccessTest);
  FRIEND_TEST(GpioCommandControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(GpioCommandControllerTest, CommandCallbackTest);
};

class GpioCommandControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();
  void SetUpHandles();

protected:
  std::unique_ptr<FriendGpioCommandController> controller_;

  // dummy gpio state values used for tests
  const std::vector<std::string> gpio_names_ = {"gpio1", "gpio2"};
  std::vector<double> gpio_commands_ = {1.0, 0.0, 3.1};
  std::vector<double> gpio_states_ = {1.0, 0.0, 3.1};

  CommandInterface gpio_1_1_dig_cmd_{gpio_names_[0], "dig.1", &gpio_commands_[0]};
  CommandInterface gpio_1_2_dig_cmd_{gpio_names_[0], "dig.2", &gpio_commands_[1]};
  CommandInterface gpio_2_ana_cmd_{gpio_names_[1], "ana.1", &gpio_commands_[2]};

  StateInterface gpio_1_1_dig_state_{gpio_names_[0], "dig.1", &gpio_states_[0]};
  StateInterface gpio_1_2_dig_state_{gpio_names_[0], "dig.2", &gpio_states_[1]};
  StateInterface gpio_2_ana_state_{gpio_names_[1], "ana.1", &gpio_states_[2]};
};

#endif  // TEST_GPIO_COMMAND_CONTROLLER_HPP_
