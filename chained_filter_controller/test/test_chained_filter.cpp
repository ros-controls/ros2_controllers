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

#include "test_chained_filter.hpp"

#include <gmock/gmock.h>

#include <memory>
#include <utility>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chained_filter_controller/chained_filter.hpp"

using chained_filter_controller::ChainedFilter;
using controller_interface::CallbackReturn;

using hardware_interface::LoanedStateInterface;

void ChainedFilterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void ChainedFilterTest::TearDownTestCase() { rclcpp::shutdown(); }

void ChainedFilterTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendChainedFilter>();
}

void ChainedFilterTest::TearDown() { controller_.reset(nullptr); }

void ChainedFilterTest::SetUpController(const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  const auto result = controller_->init("test_chained_filter", "", 0, "", node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(joint_1_pos_);
  controller_->assign_interfaces({}, std::move(state_ifs));
  executor.add_node(controller_->get_node()->get_node_base_interface());
}

TEST_F(ChainedFilterTest, InitReturnsSuccess)
{
  SetUpController(
    {rclcpp::Parameter("input_interface", std::string("wheel_left/position")),
     rclcpp::Parameter("output_interface", std::string("wheel_left/position/filtered"))});
}

TEST_F(ChainedFilterTest, InitFailureWithNoParams)
{
  const auto result =
    controller_->init("test_chained_filter", "", 0, "", controller_->define_custom_node_options());
  EXPECT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(ChainedFilterTest, ActivateReturnsSuccessWithoutError)
{
  SetUpController(
    {rclcpp::Parameter("input_interface", std::string("wheel_left/position")),
     rclcpp::Parameter("output_interface", std::string("wheel_left/position/filtered"))});

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(configure_result, CallbackReturn::SUCCESS);  // Expected because no params loaded

  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(activate_result, CallbackReturn::SUCCESS);

  ASSERT_FALSE(controller_->is_in_chained_mode())
    << "No controller is claiming the reference interfaces (it has none).";
}

TEST_F(ChainedFilterTest, UpdateFilter)
{
  // TODO(anyone): Implement a test that checks if the filter updates correctly.
}

TEST_F(ChainedFilterTest, DeactivateDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_deactivate(rclcpp_lifecycle::State()); });
}

TEST_F(ChainedFilterTest, CleanupDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_cleanup(rclcpp_lifecycle::State()); });
}

TEST_F(ChainedFilterTest, ShutdownDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_shutdown(rclcpp_lifecycle::State()); });
}
