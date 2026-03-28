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

// Tests for lifecycle methods (on_activate / on_deactivate) and the
// CANCELING → HALTED → IDLE recovery path in update().
//
// Configuration: the standard setup_parameters() fixture with
// possible_engaged_states = {"close_empty", "close_full"}.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure and activate the controller.  The initial hardware state (one of
// "open", "close_empty", "close_full") determines which state interface values
// are pre-loaded so that on_activate() can recognise the current tool state.
void prepare_for_lifecycle(
  IOGripperControllerFixture<TestableGpioToolController> & fx,
  const std::vector<std::string> & possible_states, const std::string & initial_hw_state = "open")
{
  fx.SetUpController(
    "test_gpio_tool_controller", {rclcpp::Parameter("possible_engaged_states", possible_states)});
  fx.setup_parameters();
  ASSERT_EQ(
    fx.controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  fx.SetupInterfaces();
  fx.SetInitialHardwareState(initial_hw_state);
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}
}  // namespace

// ---------------------------------------------------------------------------
// on_deactivate() always returns SUCCESS
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, OnDeactivateReturnsSuccess)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");

  EXPECT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

// ---------------------------------------------------------------------------
// on_deactivate() resets joint_states_values_ to NaN
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, OnDeactivateResetsJointStatesToNaN)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");

  controller_->on_deactivate(rclcpp_lifecycle::State());

  for (const double val : controller_->get_joint_states_values())
  {
    EXPECT_TRUE(std::isnan(val));
  }
}

// ---------------------------------------------------------------------------
// on_activate() when hardware state cannot be determined → CANCELING
//
// All state interface values remain at 0.0 (none of the configured state
// patterns match), so check_tool_state() cannot identify a tool state and
// stores ToolAction::CANCELING.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, OnActivateWithAmbiguousStateGoesToCanceling)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetupInterfaces();
  // Do NOT call SetInitialHardwareState – all values stay 0.0, no state matches.

  // on_activate returns SUCCESS even though the state is ambiguous (the FAILURE
  // path is intentionally disabled in the controller source).
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::CANCELING);
}

// ---------------------------------------------------------------------------
// update() in CANCELING → sets transition to HALTED
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, CancelingUpdateSetsHaltedTransition)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");

  controller_->force_canceling();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::CANCELING);

  UpdateController(0.0);

  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}

// ---------------------------------------------------------------------------
// HALTED + reset_halted flag → recovery to IDLE in the next update()
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, HaltedRecoveryWithResetFlag)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");

  controller_->force_canceling();
  UpdateController(0.0);  // CANCELING → HALTED
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);

  controller_->trigger_reset_halted();
  UpdateController(0.0);  // HALTED + reset → IDLE

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::IDLE);
}

// ---------------------------------------------------------------------------
// HALTED without reset_halted stays in HALTED across multiple update() calls
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, HaltedStaysHaltedWithoutResetFlag)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");

  controller_->force_canceling();
  UpdateController(0.0);  // CANCELING → HALTED
  ASSERT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);

  UpdateController(0.1);
  UpdateController(0.2);

  // Still HALTED – reset_halted_ was never set
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::HALTED);
}

// ---------------------------------------------------------------------------
// IDLE update() does NOT trigger CANCELING when the tool state is known
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, UpdateIdleWithKnownStateRemainsIdle)
{
  prepare_for_lifecycle(*this, possible_engaged_states, "open");
  ASSERT_EQ(controller_->get_current_action(), ToolAction::IDLE);

  UpdateController(0.0);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// configuration_control_enabled_=true but no configuration signal matches any
// known configuration → check_tool_state() stores CANCELING
//
// Tool state IS determinable ("open"), but with all configuration signals at 0.0
// neither narrow_objects (needs Narrow_Configuraiton_Signal=1.0) nor wide_objects
// (needs Wide_Configuration_Signal=1.0) is confirmed.
// Covers gpio_tool_controller.cpp lines 1408-1414.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, ConfigurationUndeterminedGoesToCanceling)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter(
       "configuration_joints", std::vector<std::string>{"gripper_distance_joint"})});
  setup_parameters_with_config();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetupInterfaces();
  // Tool state is known ("open"): Opened_signal=1, Closed_signal=0
  SetInitialHardwareState("open");
  // Configuration signals stay at 0.0 (default):
  //   Narrow_Configuraiton_Signal=0 → narrow_objects not matched
  //   Wide_Configuration_Signal=0  → wide_objects  not matched
  // → configuration undetermined → CANCELING

  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::CANCELING);
  // Tool state was determined correctly despite config being ambiguous
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// When configuration_control_enabled_=false (configurations=[]) the controller
// does NOT go to CANCELING even though no configuration signals are set.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerLifecycleTest, ConfigurationControlDisabledSkipsConfigCheck)
{
  // setup_parameters() sets configurations=[], so configuration_control_enabled_=false
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

  // Configuration signals are all 0.0, but since configuration_control_enabled_=false
  // the controller must NOT enter CANCELING for that reason.
  EXPECT_NE(controller_->get_current_action(), ToolAction::CANCELING);
  EXPECT_EQ(controller_->get_current_state(), "open");
}
