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

// Tests verifying that prepare_publishers_and_services() creates the correct
// set of services and action servers depending on `use_action` and whether
// configuration control is enabled.
//
// Relevant logic (from gpio_tool_controller.cpp):
//   use_action=false → disengaged_service_, engaged_service_ created;
//                      engaging_action_server_ NOT created.
//   use_action=true  → engaging_action_server_ created;
//                      disengaged_service_, engaged_service_ NOT created.
//   reset_service_ is ALWAYS created regardless of use_action.
//   reconfigure_tool_service_ created only when use_action=false AND
//     configuration_control_enabled_=true.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "test_gpio_tool_controller.hpp"

// ---------------------------------------------------------------------------
// use_action=false → disengage and engage services created, no action server
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, UseActionFalseCreatesDisengagedAndEngagedServices)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();  // sets use_action=true; we override below
  controller_->get_node()->set_parameter({"use_action", false});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(controller_->has_disengaged_service());
  EXPECT_TRUE(controller_->has_engaged_service());
  EXPECT_FALSE(controller_->has_action_server());
}

// ---------------------------------------------------------------------------
// use_action=true → action server created, no disengage/engage services
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, UseActionTrueCreatesActionServer)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter("use_action", true)});
  setup_parameters();  // also sets use_action=true

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(controller_->has_action_server());
  EXPECT_FALSE(controller_->has_disengaged_service());
  EXPECT_FALSE(controller_->has_engaged_service());
}

// ---------------------------------------------------------------------------
// reset_service_ is always created in service mode (use_action=false)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, ResetServiceAlwaysCreatedInServiceMode)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();
  controller_->get_node()->set_parameter({"use_action", false});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(controller_->has_reset_service());
}

// ---------------------------------------------------------------------------
// reset_service_ is always created in action mode (use_action=true)
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, ResetServiceAlwaysCreatedInActionMode)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter("use_action", true)});
  setup_parameters();

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(controller_->has_reset_service());
}

// ---------------------------------------------------------------------------
// use_action=false + configurations enabled → reconfigure service created
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, ReconfigureServiceCreatedWhenConfigEnabled)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter(
       "configuration_joints", std::vector<std::string>{"gripper_distance_joint"})});
  setup_parameters_with_config();
  controller_->get_node()->set_parameter({"use_action", false});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(controller_->has_reconfigure_service());
}

// ---------------------------------------------------------------------------
// use_action=false + no configurations → reconfigure service NOT created
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerServiceModeTest, NoReconfigureServiceWhenConfigDisabled)
{
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states)});
  setup_parameters();  // configurations=[] → configuration_control_enabled_=false
  controller_->get_node()->set_parameter({"use_action", false});

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  EXPECT_FALSE(controller_->has_reconfigure_service());
}
