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

  ASSERT_TRUE(controller_->params_.tf_prefix.empty());

  ASSERT_TRUE(configure_succeeds(controller_));
}

TEST_F(MotionPrimitivesForwardControllerTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), command_values_.size());
  for (size_t i = 0; i < command_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(
      command_interfaces.names[i], interface_namespace_ + "/" + command_interface_names_[i]);
  }

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), state_values_.size());
  for (size_t i = 0; i < state_interfaces.names.size(); ++i)
  {
    EXPECT_EQ(state_interfaces.names[i], interface_namespace_ + "/" + state_interface_names_[i]);
  }
}

TEST_F(MotionPrimitivesForwardControllerTest, activate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  // check that the message queue is reset
  auto & moprim_queue_ = controller_->moprim_queue_;
  ASSERT_TRUE(moprim_queue_.empty());
}

TEST_F(MotionPrimitivesForwardControllerTest, update_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));
}

TEST_F(MotionPrimitivesForwardControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(deactivate_succeeds(controller_));

  auto val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_TRUE(activate_succeeds(controller_));

  val_opt = controller_->command_interfaces_[0].get_optional();
  ASSERT_TRUE(std::isnan(val_opt.value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesForwardControllerTest, receive_single_action_goal)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  send_single_motion_sequence_goal();  // filling with default values from the function

  // Wait for the command value to be set
  // This is necessary because the action server might take some time to process the goal
  auto start = std::chrono::steady_clock::now();
  while (std::isnan(controller_->command_interfaces_[1].get_optional().value()) &&
         (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT));  // motion type
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0.1);        // q1 - q6
  EXPECT_EQ(controller_->command_interfaces_[2].get_optional().value(), 0.2);
  EXPECT_EQ(controller_->command_interfaces_[3].get_optional().value(), 0.3);
  EXPECT_EQ(controller_->command_interfaces_[4].get_optional().value(), 0.4);
  EXPECT_EQ(controller_->command_interfaces_[5].get_optional().value(), 0.5);
  EXPECT_EQ(controller_->command_interfaces_[6].get_optional().value(), 0.6);
  EXPECT_EQ(controller_->command_interfaces_[21].get_optional().value(), 3.0);  // blend radius
  EXPECT_EQ(controller_->command_interfaces_[22].get_optional().value(), 0.7);  // velocity
  EXPECT_EQ(controller_->command_interfaces_[23].get_optional().value(), 1.0);  // acceleration
  EXPECT_EQ(controller_->command_interfaces_[24].get_optional().value(), 2.0);  // move time
}

TEST_F(
  MotionPrimitivesForwardControllerTest, aborts_active_goal_and_stops_hardware_on_execution_error)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  MotionPrimitive primitive;
  primitive.type = static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT);
  primitive.joint_positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  const auto goal_handle = send_motion_sequence_goal({primitive});
  ASSERT_NE(goal_handle, nullptr);

  // simulate the hardware interface reporting an execution error
  std::ignore = state_itfs_[0]->set_value(
    static_cast<double>(motion_primitives_controllers::ExecutionState::ERROR));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // the queued/in-flight command must be cancelled on the hw-interface
  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<double>(motion_primitives_controllers::MotionHelperType::STOP_MOTION));
  for (size_t i = 1; i < command_values_.size(); ++i)
  {
    expect_command_interface_is_nan(i);
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped_result.result->error_code, -2);
}

TEST_F(MotionPrimitivesForwardControllerTest, accepts_new_goal_after_execution_error_is_handled)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));

  MotionPrimitive primitive;
  primitive.type = static_cast<uint8_t>(motion_primitives_controllers::MotionType::LINEAR_JOINT);
  primitive.joint_positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  ASSERT_NE(send_motion_sequence_goal({primitive}), nullptr);

  std::ignore = state_itfs_[0]->set_value(
    static_cast<double>(motion_primitives_controllers::ExecutionState::ERROR));
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<double>(motion_primitives_controllers::MotionHelperType::STOP_MOTION));
  std::ignore = state_itfs_[0]->set_value(
    static_cast<double>(
      motion_primitives_controllers::ExecutionState::STOPPING));  // Emulate response from hw
                                                                  // interface
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  std::ignore = state_itfs_[0]->set_value(
    static_cast<double>(
      motion_primitives_controllers::ExecutionState::STOPPED));  // Emulate response from hw
                                                                 // interface
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    controller_->command_interfaces_[0].get_optional().value(),
    static_cast<double>(motion_primitives_controllers::MotionHelperType::RESET_STOP));
  std::ignore = state_itfs_[0]->set_value(
    static_cast<double>(
      motion_primitives_controllers::ExecutionState::IDLE));  // Emulate response from hw interface

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  // aborting the previous goal must clear has_active_goal_, allowing a new goal to be accepted
  EXPECT_NE(send_motion_sequence_goal({primitive}), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
