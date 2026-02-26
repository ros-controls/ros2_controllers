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

// Tests for the ENGAGING (close) state machine transitions.
//
// Gripper parameter setup (from setup_parameters()):
//   engaged.name = "engaged"
//   SET_BEFORE_COMMAND : Release_Break_valve=1.0, Release_Something=0.0
//   CHECK_BEFORE_COMMAND (state) : Break_Engaged==0.0
//   SET_COMMAND       : Close_valve=1.0, Open_valve=0.0
//   CHECK_COMMAND (state) : Closed_signal==1.0, Part_Grasped_signal==0.0 → "close_empty"
//                           OR Closed_signal==0.0, Part_Grasped_signal==1.0 → "close_full"
//   SET_AFTER_COMMAND : Release_Something=1.0, Release_Break_valve=0.0
//   CHECK_AFTER_COMMAND (state) : Break_Engaged==1.0

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure, assign interfaces, and put controller in ENGAGING state.
// State interface values default to 0.0 so CHECK_BEFORE_COMMAND (Break_Engaged==0.0)
// passes immediately on the first check.
void prepare_for_engaging(
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

  // Place hardware in "open" so on_activate() can identify an initial state.
  fx.SetInitialHardwareState("open");
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Override whatever on_activate() set and start the ENGAGING transition.
  fx.controller_->start_engaging();
  ASSERT_EQ(fx.controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}
}  // namespace

// ---------------------------------------------------------------------------
// SET_BEFORE_COMMAND: first update() sends the "before" commands (release brake)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingSetBeforeCommandsOnFirstUpdate)
{
  prepare_for_engaging(*this, possible_engaged_states);

  ASSERT_EQ(UpdateController(0.0), controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Break_valve"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Something"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_BEFORE_COMMAND: default Break_Engaged==0.0 satisfies the condition → SET_COMMAND
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingCheckBeforeStatePassesWithDefaultZero)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND (Break_Engaged==0.0 ✓)
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_BEFORE_COMMAND: stays waiting when Break_Engaged does not match (1.0 ≠ 0.0)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingWaitsWhenBeforeStateDoesNotMatch)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND

  // Simulate brake not released: Break_Engaged==1.0 (expected 0.0)
  SetStateValue("Break_Engaged", 1.0);

  UpdateController(0.0);  // still waiting
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// SET_COMMAND: sends the main close-valve commands
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingSetMainCloseCommands)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  EXPECT_DOUBLE_EQ(GetCmdValue("Close_valve"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Open_valve"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND: keeps waiting while neither close state is confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingWaitsUntilCloseStateIsConfirmed)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Closed_signal==0.0, Part_Grasped_signal==0.0 – neither state matches
  ASSERT_DOUBLE_EQ(GetStateValue("Closed_signal"), 0.0);
  ASSERT_DOUBLE_EQ(GetStateValue("Part_Grasped_signal"), 0.0);

  UpdateController(0.0);  // still in CHECK_COMMAND
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND → SET_AFTER_COMMAND: hardware confirms close_empty position
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingProceedsToAfterCommandsWhenCloseEmptyConfirmed)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  // Simulate hardware confirming the gripper is close_empty (no part grasped)
  SetStateValue("Closed_signal", 1.0);
  SetStateValue("Part_Grasped_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);
  EXPECT_EQ(controller_->get_current_state(), "close_empty");
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND → SET_AFTER_COMMAND: hardware confirms close_full position
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingProceedsToAfterCommandsWhenCloseFullConfirmed)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  // Simulate hardware confirming the gripper closed with part grasped
  SetStateValue("Closed_signal", 0.0);
  SetStateValue("Part_Grasped_signal", 1.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);
  EXPECT_EQ(controller_->get_current_state(), "close_full");
}

// ---------------------------------------------------------------------------
// SET_AFTER_COMMAND: sends the commands to re-engage the brake (close_empty case)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingSetAfterCommandsAfterCloseEmptyConfirmed)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Closed_signal", 1.0);
  SetStateValue("Part_Grasped_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);

  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND

  // After closing (close_empty): enable Something=1, release brake back to 0
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Something"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Release_Break_valve"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_AFTER_COMMAND: waits until Break_Engaged == 1.0
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingWaitsUntilAfterStateIsConfirmed)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Closed_signal", 1.0);
  SetStateValue("Part_Grasped_signal", 0.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  // Break_Engaged is still 0.0 (brake not yet re-engaged)
  ASSERT_DOUBLE_EQ(GetStateValue("Break_Engaged"), 0.0);

  UpdateController(0.0);  // still waiting
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// Full happy-path (close_empty): all 6 steps → IDLE with state "close_empty"
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingFullTransitionToCloseEmptyCompletesSuccessfully)
{
  prepare_for_engaging(*this, possible_engaged_states);

  // 1. SET_BEFORE_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  // 2. CHECK_BEFORE_COMMAND (Break_Engaged==0.0 ✓)
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  // 3. SET_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
  EXPECT_DOUBLE_EQ(GetCmdValue("Close_valve"), 1.0);

  // Simulate gripper closing empty (no part grasped)
  SetStateValue("Closed_signal", 1.0);
  SetStateValue("Part_Grasped_signal", 0.0);

  // 4. CHECK_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);
  ASSERT_EQ(controller_->get_current_state(), "close_empty");

  // 5. SET_AFTER_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  // Simulate brake re-engagement
  SetStateValue("Break_Engaged", 1.0);

  // 6. CHECK_AFTER_COMMAND → done
  UpdateController(0.0);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "close_empty");
}

// ---------------------------------------------------------------------------
// Full happy-path (close_full): all 6 steps → IDLE with state "close_full"
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingFullTransitionToCloseFullCompletesSuccessfully)
{
  prepare_for_engaging(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  // Simulate gripper closed with part grasped
  SetStateValue("Closed_signal", 0.0);
  SetStateValue("Part_Grasped_signal", 1.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_state(), "close_full");

  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND

  // Simulate brake re-engagement
  SetStateValue("Break_Engaged", 1.0);

  UpdateController(0.0);  // CHECK_AFTER_COMMAND → IDLE

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "close_full");
}

// ---------------------------------------------------------------------------
// Timeout: CHECK_COMMAND halts if neither close state is ever confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingGoesToHaltedOnCheckCommandTimeout)
{
  prepare_for_engaging(*this, possible_engaged_states);

  // SET_BEFORE_COMMAND at t=0 → state_change_start_ = 0
  UpdateController(0.0);
  // CHECK_BEFORE_COMMAND at t=0 → SET_COMMAND
  UpdateController(0.0);
  // SET_COMMAND at t=0 → CHECK_COMMAND; state_change_start_ remains 0 from SET_BEFORE_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Do NOT set any signals. Advance time past the 5 s timeout.
  UpdateController(6.0);  // (6.0 - 0.0) > 5.0 → HALTED

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}

// ---------------------------------------------------------------------------
// Timeout: CHECK_BEFORE_COMMAND halts if the before state is never confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerCloseTest, EngagingGoesToHaltedOnCheckBeforeStateTimeout)
{
  prepare_for_engaging(*this, possible_engaged_states);

  // SET_BEFORE_COMMAND at t=0 → state_change_start_ = 0
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  // Break_Engaged == 1.0, but expected 0.0 → never confirms
  SetStateValue("Break_Engaged", 1.0);

  // Advance past the 5 s timeout
  UpdateController(6.0);

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}
