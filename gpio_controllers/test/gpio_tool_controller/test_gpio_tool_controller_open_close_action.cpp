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

// Tests for process_engaging_request() validation logic.
//
// Uses call_process_engaging_request() to directly exercise the acceptance and
// rejection conditions without going through the service/action layer.
// Relevant logic (from gpio_tool_controller.cpp):
//   - RECONFIGURING → reject with success=false
//   - already executing same action → reject with success=false
//   - executing opposite action → accept and switch (success=true)
//   - IDLE + already in target state → accept with success=true, no action started
//   - IDLE + not in target state → accept and start action (success=true)

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

namespace
{
// Configure and activate, returning with the controller in IDLE state
// and the tool in the given hardware state.
void prepare_for_request(
  IOGripperControllerFixture<TestableGpioToolController> & fx,
  const std::vector<std::string> & possible_states, const std::string & initial_hw_state)
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
// Already in disengaged ("open") state → DISENGAGING request returns success
// without starting any action (tool is already where it should be).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RequestDisengageWhenAlreadyDisengaged)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  ASSERT_EQ(controller_->get_current_state(), "open");

  auto resp = controller_->call_process_engaging_request(ToolAction::DISENGAGING, "open");

  EXPECT_TRUE(resp.success);
  // No action started – still IDLE
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Already in engaged (close_empty) state → ENGAGING request returns success
// without starting any action.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RequestEngageWhenAlreadyEngaged)
{
  prepare_for_request(*this, possible_engaged_states, "close_empty");
  ASSERT_EQ(controller_->get_current_state(), "close_empty");

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_TRUE(resp.success);
  // No action started – still IDLE
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// IDLE + in "open" state → ENGAGING request starts the action
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RequestEngageStartsActionWhenDisengaged)
{
  prepare_for_request(*this, possible_engaged_states, "open");

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// IDLE + in "close_empty" state → DISENGAGING request starts the action
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RequestDisengageStartsActionWhenEngaged)
{
  prepare_for_request(*this, possible_engaged_states, "close_empty");

  auto resp = controller_->call_process_engaging_request(ToolAction::DISENGAGING, "open");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}

// ---------------------------------------------------------------------------
// Already ENGAGING → second ENGAGING request is rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RejectsEngageWhenAlreadyEngaging)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  controller_->start_engaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
}

// ---------------------------------------------------------------------------
// Already DISENGAGING → second DISENGAGING request is rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RejectsDisengageWhenAlreadyDisengaging)
{
  prepare_for_request(*this, possible_engaged_states, "close_empty");
  controller_->start_disengaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);

  auto resp = controller_->call_process_engaging_request(ToolAction::DISENGAGING, "open");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);
}

// ---------------------------------------------------------------------------
// Currently ENGAGING → DISENGAGING request is accepted (action switches)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, AcceptsDisengageWhileEngaging)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  controller_->start_engaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);

  auto resp = controller_->call_process_engaging_request(ToolAction::DISENGAGING, "open");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);
}

// ---------------------------------------------------------------------------
// Currently DISENGAGING → ENGAGING request is accepted (action switches)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, AcceptsEngageWhileDisengaging)
{
  prepare_for_request(*this, possible_engaged_states, "close_empty");
  controller_->start_disengaging();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::DISENGAGING);

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
}

// ---------------------------------------------------------------------------
// RECONFIGURING → ENGAGING request is rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RejectsEngageWhenReconfiguring)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  controller_->start_reconfiguring("narrow_objects");
  ASSERT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);
}

// ---------------------------------------------------------------------------
// RECONFIGURING → DISENGAGING request is rejected
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, RejectsDisengageWhenReconfiguring)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  controller_->start_reconfiguring("narrow_objects");
  ASSERT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);

  auto resp = controller_->call_process_engaging_request(ToolAction::DISENGAGING, "open");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::RECONFIGURING);
}

// ---------------------------------------------------------------------------
// CANCELING → ENGAGING request is accepted and overrides the cancel
//
// CANCELING is not RECONFIGURING (first guard passes) and not IDLE (second
// guard triggers).  Because CANCELING != ENGAGING the "already same action"
// early-return is NOT taken, so the function falls through and starts the
// new ENGAGING action.  This covers the CANCELING branch of the
// current_tool_action_ != IDLE path (gpio_tool_controller.cpp line 929).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerRequestTest, ProcessEngagingRequestDuringCancelingAcceptsNewAction)
{
  prepare_for_request(*this, possible_engaged_states, "open");
  controller_->force_canceling();
  ASSERT_EQ(controller_->get_current_action(), ToolAction::CANCELING);

  auto resp = controller_->call_process_engaging_request(ToolAction::ENGAGING, "engaged");

  EXPECT_TRUE(resp.success);
  // CANCELING is overridden by the new ENGAGING action
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}
