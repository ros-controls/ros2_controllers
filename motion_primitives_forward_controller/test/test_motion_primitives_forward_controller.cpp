// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_motion_primitives_forward_controller.hpp"

class MotionPrimitivesForwardControllerTest
: public MotionPrimitivesForwardControllerFixture<TestableMotionPrimitivesForwardController>
{
};

TEST_F(MotionPrimitivesForwardControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.command_interfaces.empty());
  ASSERT_TRUE(controller_->params_.state_interfaces.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(
    controller_->params_.command_interfaces, testing::ElementsAreArray(command_interface_names_));
  ASSERT_THAT(
    controller_->params_.state_interfaces, testing::ElementsAreArray(state_interface_names_));
}

TEST_F(MotionPrimitivesForwardControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], interface_namespace_ + "/" + command_interface_names_[i]);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], interface_namespace_ + "/" + state_interface_names_[i]);
  }
}

TEST_F(MotionPrimitivesForwardControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message queue is reset
  auto moprim_queue_ = controller_->moprim_queue_;
  ASSERT_TRUE(moprim_queue_.empty());
}

TEST_F(MotionPrimitivesForwardControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(MotionPrimitivesForwardControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, receive_single_action_goal)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  send_single_motion_sequence_goal();  // filling with default values from the function

  // Wait for the command value to be set
  // This is necessary because the action server might take some time to process the goal
  auto start = std::chrono::steady_clock::now();
  while (std::isnan(command_values_[1]) &&
         (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(
    command_values_[0],
    static_cast<uint8_t>(
      motion_primitives_forward_controller::MotionType::LINEAR_JOINT));  // motion type
  EXPECT_EQ(command_values_[1], 0.1);                                    // q1 - q6
  EXPECT_EQ(command_values_[2], 0.2);
  EXPECT_EQ(command_values_[3], 0.3);
  EXPECT_EQ(command_values_[4], 0.4);
  EXPECT_EQ(command_values_[5], 0.5);
  EXPECT_EQ(command_values_[6], 0.6);
  EXPECT_EQ(command_values_[21], 3.0);  // blend radius
  EXPECT_EQ(command_values_[22], 0.7);  // velocity
  EXPECT_EQ(command_values_[23], 1.0);  // acceleration
  EXPECT_EQ(command_values_[24], 2.0);  // move time
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
