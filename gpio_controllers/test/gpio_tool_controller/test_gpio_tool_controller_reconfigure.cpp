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

// Tests for the RECONFIGURING state machine transitions.
//
// Configuration setup (from setup_parameters_with_config()):
//   narrow_objects:
//     SET_BEFORE_COMMAND  : (none)
//     CHECK_BEFORE_COMMAND: (none)
//     SET_COMMAND         : Narrow_Configuration_Cmd=1.0, Wide_Configuration_Cmd=0.0
//     CHECK_COMMAND(state): Narrow_Configuraiton_Signal=1.0, Wide_Configuration_Signal=0.0
//     SET_AFTER_COMMAND   : (none)
//     CHECK_AFTER_COMMAND : (none)
//   wide_objects:
//     SET_COMMAND         : Narrow_Configuration_Cmd=0.0, Wide_Configuration_Cmd=1.0
//     CHECK_COMMAND(state): Narrow_Configuraiton_Signal=0.0, Wide_Configuration_Signal=1.0
//
// Because the before/after steps have no interfaces, each transitions immediately
// in a single update() call without waiting.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure, assign interfaces, and put controller in RECONFIGURING state targeting
// the given configuration name (default: "narrow_objects").
void prepare_for_reconfiguring(
  IOGripperControllerFixture<TestableGpioToolController> & fx,
  const std::vector<std::string> & possible_states,
  const std::string & target_config = "narrow_objects")
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

  // Start from "open" so on_activate() can identify an initial state.
  fx.SetInitialHardwareState("open");
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Override whatever on_activate() set and start the RECONFIGURING transition.
  fx.controller_->start_reconfiguring(target_config);
  ASSERT_EQ(fx.controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
  ASSERT_EQ(fx.controller_->get_current_action(), ToolAction::RECONFIGURING);
}
}  // namespace

// ---------------------------------------------------------------------------
// SET_BEFORE_COMMAND: narrow_objects has no before-commands → immediately
// transitions to CHECK_BEFORE_COMMAND in a single update()
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringSetBeforeCommandsTransitionsImmediately)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  ASSERT_EQ(UpdateController(0.0), controller_interface::return_type::OK);

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_BEFORE_COMMAND: no before-state conditions → immediately transitions
// to SET_COMMAND
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringCheckBeforeStateTransitionsImmediately)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);
}

// ---------------------------------------------------------------------------
// SET_COMMAND: sends the configuration switch commands
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringSetMainConfigurationCommands)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  EXPECT_DOUBLE_EQ(GetCmdValue("Narrow_Configuration_Cmd"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Wide_Configuration_Cmd"), 0.0);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND: keeps waiting while Narrow_Configuraiton_Signal is not yet 1.0
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringWaitsUntilNarrowStateIsConfirmed)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Narrow_Configuraiton_Signal is still 0.0 (hardware hasn't responded yet)
  ASSERT_DOUBLE_EQ(GetStateValue("Narrow_Configuraiton_Signal"), 0.0);

  UpdateController(0.0);  // still in CHECK_COMMAND
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_COMMAND → SET_AFTER_COMMAND: hardware confirms narrow configuration
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringProceedsToAfterCommandsWhenStateConfirmed)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  // Simulate hardware confirming narrow configuration
  SetStateValue("Narrow_Configuraiton_Signal", 1.0);
  // Wide_Configuration_Signal stays 0.0 (expected value)

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// SET_AFTER_COMMAND: narrow_objects has no after-commands → immediately
// transitions to CHECK_AFTER_COMMAND
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringSetAfterCommandsTransitionsImmediately)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Narrow_Configuraiton_Signal", 1.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);

  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);
}

// ---------------------------------------------------------------------------
// CHECK_AFTER_COMMAND: no after-state conditions → immediately transitions to IDLE
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringCheckAfterStateTransitionsImmediately)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  SetStateValue("Narrow_Configuraiton_Signal", 1.0);

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  UpdateController(0.0);  // CHECK_AFTER_COMMAND → IDLE
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
}

// ---------------------------------------------------------------------------
// Full happy-path to narrow_objects: 6 steps (all before/after are empty) → IDLE
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringToNarrowFullTransitionCompletesSuccessfully)
{
  prepare_for_reconfiguring(*this, possible_engaged_states, "narrow_objects");

  // 1. SET_BEFORE_COMMAND (empty → immediate)
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_BEFORE_COMMAND);

  // 2. CHECK_BEFORE_COMMAND (empty → immediate)
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_COMMAND);

  // 3. SET_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);
  EXPECT_DOUBLE_EQ(GetCmdValue("Narrow_Configuration_Cmd"), 1.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Wide_Configuration_Cmd"), 0.0);

  // Simulate narrow configuration confirmed by hardware
  SetStateValue("Narrow_Configuraiton_Signal", 1.0);
  // Wide_Configuration_Signal stays 0.0 (expected)

  // 4. CHECK_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_AFTER_COMMAND);

  // 5. SET_AFTER_COMMAND (empty → immediate)
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_AFTER_COMMAND);

  // 6. CHECK_AFTER_COMMAND (empty → immediate) → done
  UpdateController(0.0);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
}

// ---------------------------------------------------------------------------
// Full happy-path to wide_objects: same 6-step pattern with Wide commands/signals
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringToWideFullTransitionCompletesSuccessfully)
{
  prepare_for_reconfiguring(*this, possible_engaged_states, "wide_objects");

  UpdateController(0.0);  // SET_BEFORE_COMMAND → CHECK_BEFORE_COMMAND
  UpdateController(0.0);  // CHECK_BEFORE_COMMAND → SET_COMMAND
  UpdateController(0.0);  // SET_COMMAND → CHECK_COMMAND

  EXPECT_DOUBLE_EQ(GetCmdValue("Narrow_Configuration_Cmd"), 0.0);
  EXPECT_DOUBLE_EQ(GetCmdValue("Wide_Configuration_Cmd"), 1.0);

  // Simulate wide configuration confirmed by hardware
  SetStateValue("Wide_Configuration_Signal", 1.0);
  // Narrow_Configuraiton_Signal stays 0.0 (expected)

  UpdateController(0.0);  // CHECK_COMMAND → SET_AFTER_COMMAND
  UpdateController(0.0);  // SET_AFTER_COMMAND → CHECK_AFTER_COMMAND
  UpdateController(0.0);  // CHECK_AFTER_COMMAND → IDLE

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
}

// ---------------------------------------------------------------------------
// Timeout: CHECK_COMMAND halts if the configuration state is never confirmed
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerReconfigureTest, ReconfiguringGoesToHaltedOnCheckCommandTimeout)
{
  prepare_for_reconfiguring(*this, possible_engaged_states);

  // SET_BEFORE_COMMAND at t=0 → state_change_start_ = 0
  UpdateController(0.0);
  // CHECK_BEFORE_COMMAND at t=0 → SET_COMMAND
  UpdateController(0.0);
  // SET_COMMAND at t=0 → CHECK_COMMAND; state_change_start_ remains 0 from SET_BEFORE_COMMAND
  UpdateController(0.0);
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::CHECK_COMMAND);

  // Do NOT set Narrow_Configuraiton_Signal. Advance time past the 5 s timeout.
  UpdateController(6.0);  // (6.0 - 0.0) > 5.0 → HALTED

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}
