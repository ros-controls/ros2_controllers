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
#include "test_motion_primitives_from_trajectory_controller.hpp"

class MotionPrimitivesFromTrajectoryControllerTest
: public MotionPrimitivesFromTrajectoryControllerFixture<
    TestableMotionPrimitivesFromTrajectoryController>
{
};

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.tf_prefix.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

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

// TODO(anyone): enable /compute_fk service and re-enable the following tests that depend on it
#if 0
TEST_F(MotionPrimitivesFromTrajectoryControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message queue is reset
  auto & moprim_queue_ = controller_->moprim_queue_;
  ASSERT_TRUE(moprim_queue_.empty());
}

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, reactivate_success)
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

TEST_F(MotionPrimitivesFromTrajectoryControllerTest, receive_single_action_goal)
{
}
#endif

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
