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

// Tests the compare_exchange discipline on the packed (action, transition) state, via
// try_advance(). A concurrent writer is simulated by overwriting the packed state between
// reading the snapshot and calling try_advance() - no threads needed.

#include "test_gpio_tool_controller.hpp"

// No concurrent write between the read and the compare_exchange: it succeeds.
TEST_F(GpioToolControllerTest, UncontestedAdvanceSucceeds)
{
  controller_->set_state(ToolAction::IDLE, GPIOToolTransition::IDLE);
  uint16_t expected = controller_->get_packed_state();

  const bool advanced =
    controller_->try_advance(expected, ToolAction::ENGAGING, GPIOToolTransition::SET_BEFORE_COMMAND);

  EXPECT_TRUE(advanced);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}

// A concurrent write lands between the read and the compare_exchange: it fails and does not
// clobber the concurrent write.
TEST_F(GpioToolControllerTest, ConcurrentChangeIsNotClobbered)
{
  controller_->set_state(ToolAction::IDLE, GPIOToolTransition::IDLE);
  uint16_t stale_snapshot = controller_->get_packed_state();

  controller_->set_state(ToolAction::ENGAGING, GPIOToolTransition::SET_BEFORE_COMMAND);

  uint16_t expected = stale_snapshot;
  const bool advanced =
    controller_->try_advance(expected, ToolAction::IDLE, GPIOToolTransition::IDLE);

  EXPECT_FALSE(advanced);
  EXPECT_EQ(expected, controller_->get_packed_state());
  EXPECT_EQ(controller_->get_current_action(), ToolAction::ENGAGING);
  EXPECT_EQ(controller_->get_current_transition(), GPIOToolTransition::SET_BEFORE_COMMAND);
}
