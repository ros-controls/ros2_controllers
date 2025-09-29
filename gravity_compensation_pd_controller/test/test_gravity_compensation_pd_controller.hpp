// Copyright (c) 2025, University of Salerno, Automatic Control Group
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
//
// Authors: Davide Risi

#ifndef TEST_GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_
#define TEST_GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "gravity_compensation_pd_controller/gravity_compensation_pd_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

// Create a friend class to access private and protected members for testing
class FriendGravityCompensationPDController
: public gravity_compensation_pd_controller::GravityCompensationPDController
{
  FRIEND_TEST(GravityCompensationPDControllerTest, StopJointsOnDeactivateTest);
  FRIEND_TEST(GravityCompensationPDControllerTest, ConfigureAndActivateParamsSuccess);
  FRIEND_TEST(GravityCompensationPDControllerTest, NoCommandCheckTest);
};

class GravityCompensationPDControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController(const std::vector<rclcpp::Parameter> & parameters = {});

protected:
  void assign_interfaces_();

protected:
  std::unique_ptr<FriendGravityCompensationPDController> controller_;
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  std::vector<double> joint_command_values_ = {0.0, 0.0, 0.0};
  std::vector<double> joint_position_values_ = {0.0, 0.0, 0.0};
  std::vector<double> joint_velocity_values_ = {0.0, 0.0, 0.0};

  // Store the interfaces to keep them alive
  std::vector<std::unique_ptr<hardware_interface::CommandInterface>> command_interfaces_;
  std::vector<std::unique_ptr<hardware_interface::StateInterface>> state_interfaces_;

  rclcpp::executors::SingleThreadedExecutor executor_;
};

#endif  // TEST_GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_
