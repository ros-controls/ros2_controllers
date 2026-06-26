// Copyright 2024 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "test_forward_state_controller.hpp"

#include "forward_state_controller/forward_state_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using testing::IsEmpty;
using testing::SizeIs;

void ForwardStateControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void ForwardStateControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void ForwardStateControllerTest::SetUp()
{
  controller_ = std::make_unique<FriendForwardStateController>();
}

void ForwardStateControllerTest::TearDown() { controller_.reset(nullptr); }

void ForwardStateControllerTest::SetUpController(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "forward_state_controller";
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = node_options;
  const auto result = controller_->init(params);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint2_pos_cmd_, nullptr);
  command_ifs.emplace_back(joint3_pos_cmd_, nullptr);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint1_pos_state_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(ForwardStateControllerTest, StateInterfacesParameterNotSet)
{
  // state_interfaces is a required parameter - init itself fails when it is not provided
  auto node_options = controller_->define_custom_node_options();
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "forward_state_controller";
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = node_options;
  ASSERT_EQ(controller_->init(params), controller_interface::return_type::ERROR);
}

TEST_F(ForwardStateControllerTest, ForwardStateMappingMissing)
{
  // state_interfaces lists joint1/position but forward_state has no entry for it:
  // generate_parameter_library enforces the map at init() time, so init() itself fails
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(
    {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"joint1/position"})});
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "forward_state_controller";
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = node_options;
  ASSERT_EQ(controller_->init(params), controller_interface::return_type::ERROR);
}

TEST_F(ForwardStateControllerTest, ConfigureParamsSuccess)
{
  SetUpController(
    {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"joint1/position"}),
     rclcpp::Parameter(
       "forward_state.joint1/position.to_command", std::vector<std::string>{"joint2/position"})});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = controller_->command_interface_configuration();
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_THAT(cmd_if_conf.names, SizeIs(1lu));
  ASSERT_THAT(cmd_if_conf.names[0], "joint2/position");

  auto state_if_conf = controller_->state_interface_configuration();
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
  ASSERT_THAT(state_if_conf.names[0], "joint1/position");
}

TEST_F(ForwardStateControllerTest, ActivateSuccess)
{
  SetUpController(
    {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"joint1/position"}),
     rclcpp::Parameter(
       "forward_state.joint1/position.to_command", std::vector<std::string>{"joint2/position"})});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(ForwardStateControllerTest, UpdateForwardsStateToCommand)
{
  SetUpController(
    {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"joint1/position"}),
     rclcpp::Parameter(
       "forward_state.joint1/position.to_command", std::vector<std::string>{"joint2/position"})});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // initial state: joint1 = 5.5, joint2 cmd = 0.0
  ASSERT_DOUBLE_EQ(state_position_value_, 5.5);
  ASSERT_DOUBLE_EQ(cmd_position_value_1_, 0.0);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // joint2 cmd should now be forwarded from joint1 state
  ASSERT_DOUBLE_EQ(cmd_position_value_1_, 5.5);
}

TEST_F(ForwardStateControllerTest, UpdateForwardsOneStateToMultipleCommands)
{
  SetUpController(
    {rclcpp::Parameter("state_interfaces", std::vector<std::string>{"joint1/position"}),
     rclcpp::Parameter(
       "forward_state.joint1/position.to_command",
       std::vector<std::string>{"joint2/position", "joint3/position"})});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_DOUBLE_EQ(state_position_value_, 5.5);
  ASSERT_DOUBLE_EQ(cmd_position_value_1_, 0.0);
  ASSERT_DOUBLE_EQ(cmd_position_value_2_, 0.0);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // both command interfaces should receive the state value
  ASSERT_DOUBLE_EQ(cmd_position_value_1_, 5.5);
  ASSERT_DOUBLE_EQ(cmd_position_value_2_, 5.5);
}
