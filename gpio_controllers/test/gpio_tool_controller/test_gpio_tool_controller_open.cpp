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

// Tests for the DISENGAGING (open) state machine transitions.
//
// Gripper parameter setup (from setup_parameters()):
//   disengaged.name = "open"
//   SET_BEFORE_COMMAND : Release_Break_valve=1.0, Release_Something=0.0
//   CHECK_BEFORE_COMMAND (state) : Break_Engaged==0.0
//   SET_COMMAND       : Open_valve=1.0, Close_valve=0.0
//   CHECK_COMMAND (state) : Opened_signal==1.0, Closed_signal==0.0
//   SET_AFTER_COMMAND : Release_Break_valve=0.0, Release_Something=1.0
//   CHECK_AFTER_COMMAND (state) : Break_Engaged==1.0

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure, assign interfaces, and put controller in DISENGAGING state.
// State interface values default to 0.0 so CHECK_BEFORE_COMMAND (Break_Engaged==0.0)
// passes immediately on the first check.
void prepare_for_disengaging(
  IOGripperControllerFixture<TestableGpioToolController> & fx,
  const std::vector<std::string> & possible_states)
{
  fx.SetUpController(
    "test_gpio_tool_controller", {rclcpp::Parameter("possible_engaged_states", possible_states)});
  fx.setup_parameters();
  ASSERT_EQ(
    fx.controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  fx.SetupInterfaces();

  // Place hardware in "close_empty" so on_activate() can identify an initial state.
  fx.SetInitialHardwareState("close_empty");
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Override whatever on_activate() set and start the DISENGAGING transition.
  fx.controller_->start_disengaging();
  ASSERT_EQ(fx.controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}
}  // namespace

// ---------------------------------------------------------------------------
// SET_BEFORE_COMMAND: first update() sends the "before" commands (release brake)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingSetBeforeCommandsOnFirstUpdate)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  ASSERT_EQ(UpdateController(0.0), controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Break_valve"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Something"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_BEFORE_COMMAND: default Break_Engaged==0.0 satisfies the condition → SET_COMMAND
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingCheckBeforeStatePassesWithDefaultZero)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND (Break_Engaged==0.0 ✓)
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_BEFORE_COMMAND: stays waiting when Break_Engaged does not match (1.0 ≠ 0.0)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingWaitsWhenBeforeStateDoesNotMatch)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND

  // Simulate brake not released: Break_Engaged==1.0 (expected 0.0)
  SetStateValue("Break_Engaged", 1.0);

  UpdateController(0.0);  // still waiting
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// SET_COMMAND: sends the main open-valve commands
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingSetMainOpenCommands)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  EXPECT_DOUBLE_EQ(GetCmdValue("Open_valve"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Close_valve"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND: keeps waiting while Opened_signal is not yet 1.0
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingWaitsUntilOpenStateIsConfirmed)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Opened_signal is still 0.0 (hardware hasn't responded yet)
  ASSERT_DOUBLE_EQ(GetStateValue("Opened_signal"), 0.0);

  UpdateController(0.0);  // still in CHECK_COMMAND
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND → SET_AFTER_COMMAND: hardware confirms open position
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingProceedsToAfterCommandsWhenOpenConfirmed)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  // Simulate hardware confirming the gripper is now open
  SetStateValue("Opened_signal", 1.0);
  SetStateValue("Closed_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// SET_AFTER_COMMAND: sends the commands to re-engage the brake
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingSetAfterCommandsAfterOpenConfirmed)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Opened_signal", 1.0);
  SetStateValue("Closed_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);

  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND

  // After opening: release brake valve back to 0, enable Something=1
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Break_valve"), 0.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Something"), 1.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_AFTER_COMMAND: waits until Break_Engaged == 1.0
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingWaitsUntilAfterStateIsConfirmed)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Opened_signal", 1.0);
  SetStateValue("Closed_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  // Break_Engaged is still 0.0 (brake not yet re-engaged)
  ASSERT_DOUBLE_EQ(GetStateValue("Break_Engaged"), 0.0);

  UpdateController(0.0);  // still waiting
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// Full happy-path: all 6 steps → IDLE with state "open"
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingFullTransitionCompletesSuccessfully)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  // 1. SET_BEFORE_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  // 2. CHECK_BEFORE_COMMAND (Break_Engaged==0.0 ✓)
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  // 3. SET_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
  EXPECT_DOUBLE_EQ(GetCmdValue("Open_valve"), 1.0);

  // Simulate gripper opening
  SetStateValue("Opened_signal", 1.0);
  SetStateValue("Closed_signal", 0.0);

  // 4. CHECK_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);

  // 5. SET_AFTER_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  // Simulate brake re-engagement
  SetStateValue("Break_Engaged", 1.0);

  // 6. CHECK_AFTER_COMMAND → done
  UpdateController(0.0);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// Timeout: CHECK_COMMAND halts if the open state is never confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingGoesToHaltedOnCheckCommandTimeout)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  // SET_BEFORE_COMMAND at t=0 → state_change_start_ = 0
  UpdateController(0.0);
  // CHECK_BEFORE_COMMAND at t=0 → SET_COMMAND
  UpdateController(0.0);
  // SET_COMMAND at t=0 → CHECK_COMMAND; state_change_start_ remains 0 from SET_BEFORE_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Do NOT set Opened_signal. Advance time past the 5 s timeout.
  UpdateController(6.0);  // (6.0 - 0.0) > 5.0 → HALTED

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}

// ---------------------------------------------------------------------------
// Timeout: CHECK_BEFORE_COMMAND halts if the before state is never confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, DisengagingGoesToHaltedOnCheckBeforeStateTimeout)
{
  prepare_for_disengaging(*this, possible_engaged_states);

  // SET_BEFORE_COMMAND at t=0 → state_change_start_ = 0
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  // Break_Engaged == 1.0, but expected 0.0 → never confirms
  SetStateValue("Break_Engaged", 1.0);

  // Advance past the 5 s timeout
  UpdateController(6.0);

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}

// ---------------------------------------------------------------------------
// IDLE update does not modify commands and returns OK
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerOpenTest, UpdateInIdleStateReturnsOkAndDoesNotChangeCommands)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetupInterfaces();
  SetInitialHardwareState("open");
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->get_current_action(), ToolAction::IDLE);

  const double cmd_before = GetCmdValue("Open_valve");
  ASSERT_EQ(UpdateController(0.0), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(GetCmdValue("Open_valve"), cmd_before);
}
