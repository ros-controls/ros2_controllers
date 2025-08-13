// Copyright 2025 ros2_control PMC
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

#include "test_multiple_chained_filter.hpp"

#include <gmock/gmock.h>

#include <memory>
#include <utility>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chained_filter_controller/chained_filter.hpp"

using chained_filter_controller::ChainedFilter;
using controller_interface::CallbackReturn;
using testing::SizeIs;

using hardware_interface::LoanedStateInterface;

void MultipleChainedFilterTest::SetUpTestCase() { /*rclcpp::init(0, nullptr);*/ }

void MultipleChainedFilterTest::TearDownTestCase() { /*rclcpp::shutdown(); */ }

void MultipleChainedFilterTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<ChainedFilter>();
}

void MultipleChainedFilterTest::TearDown() { controller_.reset(nullptr); }

void MultipleChainedFilterTest::SetUpController(
  const std::string node_name, const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  const auto result = controller_->init(node_name, "", 0, "", node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_);
  state_ifs.emplace_back(joint_2_pos_);
  controller_->assign_interfaces({}, std::move(state_ifs));
  executor.add_node(controller_->get_node()->get_node_base_interface());
}

TEST_F(MultipleChainedFilterTest, InitReturnsSuccess) { SetUpController(); }

TEST_F(MultipleChainedFilterTest, ActivateReturnsSuccessWithoutError)
{
  SetUpController();

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(configure_result, CallbackReturn::SUCCESS);

  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(activate_result, CallbackReturn::SUCCESS);

  ASSERT_FALSE(controller_->is_in_chained_mode())
    << "No controller is claiming the reference interfaces (it has none).";
}

TEST_F(
  MultipleChainedFilterTest,
  state_interface_configuration_succeeds_when_multiple_wheels_are_specified)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(2u));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(0u));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);

  auto state_if_exported_conf = controller_->export_state_interfaces();
  ASSERT_THAT(state_if_exported_conf, SizeIs(2u));
  EXPECT_EQ(
    state_if_exported_conf[0]->get_name(),
    "test_chained_filter_multiple_interfaces/wheel_left/filtered_position");
  EXPECT_EQ(
    state_if_exported_conf[0]->get_prefix_name(),
    "test_chained_filter_multiple_interfaces/wheel_left");
  EXPECT_EQ(state_if_exported_conf[0]->get_interface_name(), "filtered_position");

  EXPECT_EQ(
    state_if_exported_conf[1]->get_name(),
    "test_chained_filter_multiple_interfaces/wheel_right/filtered_position");
  EXPECT_EQ(
    state_if_exported_conf[1]->get_prefix_name(),
    "test_chained_filter_multiple_interfaces/wheel_right");
  EXPECT_EQ(state_if_exported_conf[1]->get_interface_name(), "filtered_position");
}

TEST_F(MultipleChainedFilterTest, UpdateFilter_multiple_interfaces)
{
  SetUpController();
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // input state interface should not change
  EXPECT_EQ(joint_1_pos_.get_optional().value(), joint_states_[0]);
  EXPECT_EQ(joint_2_pos_.get_optional().value(), joint_states_[1]);
  // output should be the same
  auto state_if_exported_conf = controller_->export_state_interfaces();
  ASSERT_THAT(state_if_exported_conf, SizeIs(2u));
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), joint_states_[0]);
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), joint_states_[1]);

  ASSERT_TRUE(joint_1_pos_.set_value(2.0));
  ASSERT_TRUE(joint_2_pos_.set_value(3.0));
  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // input and output should have changed
  EXPECT_EQ(joint_1_pos_.get_optional().value(), joint_states_[0]);
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), 1.55);
  EXPECT_EQ(joint_2_pos_.get_optional().value(), joint_states_[1]);
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), 2.6);

  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // output should have reached steady state (mean filter)
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), joint_states_[0]);
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), joint_states_[1]);
}

TEST_F(MultipleChainedFilterTest, UpdateFilter_multiple_interfaces_config_per_input)
{
  SetUpController("test_chained_filter_multiple_interfaces_config_per_input");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // input state interface should not change
  EXPECT_EQ(joint_1_pos_.get_optional().value(), joint_states_[0]);
  EXPECT_EQ(joint_2_pos_.get_optional().value(), joint_states_[1]);
  // output should be the same
  auto state_if_exported_conf = controller_->export_state_interfaces();
  ASSERT_THAT(state_if_exported_conf, SizeIs(2u));
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), joint_states_[0]);
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), joint_states_[1]);

  ASSERT_TRUE(joint_1_pos_.set_value(2.0));
  ASSERT_TRUE(joint_2_pos_.set_value(2.8));
  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // input and output should have changed
  EXPECT_EQ(joint_1_pos_.get_optional().value(), joint_states_[0]);
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), 1.55);
  EXPECT_EQ(joint_2_pos_.get_optional().value(), joint_states_[1]);
  // second update call, mean of (2.2, 2.8)
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), 2.5);

  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // output should have reached steady state for the first input (mean filter)
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), joint_states_[0]);
  // third update call, mean of (2.2, 2.8, 2.8)
  EXPECT_EQ(state_if_exported_conf[1]->get_optional().value(), 2.6);

  ASSERT_EQ(
    controller_->update_and_write_commands(rclcpp::Time(), rclcpp::Duration::from_seconds(0.1)),
    controller_interface::return_type::OK);
  // no change
  EXPECT_EQ(state_if_exported_conf[0]->get_optional().value(), joint_states_[0]);
  // output should have reached steady state (mean filter)
  EXPECT_NEAR(state_if_exported_conf[1]->get_optional().value(), joint_states_[1], 1e-5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
