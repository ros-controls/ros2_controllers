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

#include <stddef.h>

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "force_torque_sensor_controller/force_torque_sensor_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_force_torque_sensor_controller.hpp"

using hardware_interface::LoanedStateInterface;
using std::placeholders::_1;
using testing::Each;
using testing::ElementsAreArray;
using testing::IsEmpty;
using testing::SizeIs;


namespace
{
constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

}  // namespace

void ForceTorqueSensorControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void ForceTorqueSensorControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void ForceTorqueSensorControllerTest::SetUp()
{
  // initialize controller
  state_controller_ = std::make_unique<FriendForceTorqueSensorController>();
}

void ForceTorqueSensorControllerTest::TearDown()
{
  state_controller_.reset(nullptr);
}

void ForceTorqueSensorControllerTest::SetUpStateController()
{
  const auto result = state_controller_->init("force_torque_sensor_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(sensor_1_fx_state_);
  state_ifs.emplace_back(sensor_2_fx_state_);
  state_ifs.emplace_back(sensor_3_fx_state_);
  state_ifs.emplace_back(sensor_1_tz_state_);
  state_ifs.emplace_back(sensor_2_tz_state_);
  state_ifs.emplace_back(sensor_3_tz_state_);

  state_controller_->assign_interfaces({}, std::move(state_ifs));
}

TEST_F(ForceTorqueSensorControllerTest, SensorNameParameterNotSet)
{
  SetUpStateController();

  // configure failed, 'sensor_name' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, InterfaceNamesParameterNotSet)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // configure failed, 'interface_names' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, FrameIdParameterNotSet)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, SensorNameParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name' empty
  state_controller_->get_node()->set_parameter({"sensor_name", ""});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // set the 'frame_id'
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure failed, 'sensor_name' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, InterfaceNameParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names' empty
  state_controller_->get_node()->set_parameter({"interface_names", std::vector<std::string>()});

  // set the 'frame_id'
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure failed, 'interface_name' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, FrameIdParameterIsEmpty)
{
  SetUpStateController();

  // set the 'sensor_name'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});

  // set the 'interface_names'
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});

  // set the 'frame_id' empty
  state_controller_->get_node()->set_parameter({"frame_id", ""});

  // configure failed, 'frame_id' parameter empty
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorControllerTest, ConfigureParamsSuccess)
{
  SetUpStateController();

  // set the params 'sensor_name', 'interface_names' and 'frame_id'
  state_controller_->get_node()->set_parameter({"sensor_name", "dummy"});
  state_controller_->get_node()->set_parameter(
    {"interface_names",
      std::vector<std::string>{"fx", "tz"}});
  state_controller_->get_node()->set_parameter({"frame_id", "dummy_frame"});

  // configure success
  ASSERT_EQ(state_controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}
