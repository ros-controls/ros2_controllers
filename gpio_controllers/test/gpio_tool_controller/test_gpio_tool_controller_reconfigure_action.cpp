// Copyright (c) 2025, b»robotized by Stogl Robotics
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

// Tests for process_reconfigure_request() validation logic.
//
// Uses call_process_reconfigure_request() to directly exercise the acceptance
// and rejection conditions without going through the service/action layer.
// Relevant logic (from gpio_tool_controller.cpp):
//   - Empty config name → reject
//   - Unknown config name (not in configurations list) → reject
//   - Tool not IDLE (busy with another action) → reject
//   - Tool not in disengaged state ("open") → reject
//   - IDLE + disengaged + valid name → accept, start RECONFIGURING

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure and activate with configuration-enabled setup. The controller
// will be in IDLE state and the tool in the given hardware state.
void prepare_for_reconfigure_request(
  IOGripperControllerFixture<TestableGpioToolController> & fx,
  const std::vector<std::string> & possible_states, const std::string & initial_hw_state = "open")
{
  fx.SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter(
       "configuration_joints", std::vector<std::string>{"gripper_distance_joint"})});
  fx.setup_parameters_with_config();
  ASSERT_EQ(
    fx.controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  fx.SetupInterfaces();
  fx.SetInitialHardwareState(initial_hw_state);
  // Also set a known configuration so check_tool_state() can determine current_configuration_.
  // Default to narrow_objects (Narrow_Configuraiton_Signal=1.0, Wide_Configuration_Signal=0.0).
  fx.SetStateValue("Narrow_Configuraiton_Signal", 1.0);
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
}  // namespace

// ---------------------------------------------------------------------------
// IDLE + disengaged ("open") + valid config name → reconfiguration starts
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, AcceptsReconfigureWhenIdleAndDisengaged)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "open");
  ASSERT_EQ(controller_->get_current_state(), "open");

  auto resp = controller_->call_process_reconfigure_request("narrow_objects");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// Empty config name → rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWithEmptyConfigName)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "open");

  auto resp = controller_->call_process_reconfigure_request("");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Unknown config name (not in configurations list) → rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWithUnknownConfigName)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "open");

  auto resp = controller_->call_process_reconfigure_request("nonexistent_config");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Busy (ENGAGING) → reconfiguration rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWhenEngaging)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "open");
  controller_->start_engaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);

  auto resp = controller_->call_process_reconfigure_request("narrow_objects");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
}

// ---------------------------------------------------------------------------
// Busy (DISENGAGING) → reconfiguration rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWhenDisengaging)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "close_empty");
  controller_->start_disengaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);

  auto resp = controller_->call_process_reconfigure_request("narrow_objects");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);
}

// ---------------------------------------------------------------------------
// IDLE but tool is in engaged state (close_empty) → reconfiguration rejected
// (tool must be disengaged/"open" to reconfigure)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWhenNotDisengaged)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "close_empty");
  ASSERT_EQ(controller_->get_current_state(), "close_empty");

  auto resp = controller_->call_process_reconfigure_request("narrow_objects");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Already RECONFIGURING → second reconfigure request is rejected
//
// Covers the `current_tool_action_.load() != ToolAction::IDLE` guard inside
// process_reconfigure_request() (gpio_tool_controller.cpp line 994).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, RejectsReconfigureWhenAlreadyReconfiguring)
{
  prepare_for_reconfigure_request(*this, possible_engaged_states, "open");
  ASSERT_EQ(controller_->get_current_state(), "open");

  controller_->start_reconfiguring("narrow_objects");
  ASSERT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);

  auto resp = controller_->call_process_reconfigure_request("wide_objects");

  EXPECT_FALSE(resp.success);
  // Action must not change
  EXPECT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);
}
