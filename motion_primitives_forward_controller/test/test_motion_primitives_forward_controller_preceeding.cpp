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

#include "test_motion_primitives_forward_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using motion_primitives_forward_controller::CMD_MY_ITFS;
using motion_primitives_forward_controller::STATE_MY_ITFS;

class MotionPrimitivesForwardControllerTest : public MotionPrimitivesForwardControllerFixture<TestableMotionPrimitivesForwardController>
{
};

TEST_F(MotionPrimitivesForwardControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.command_interfaces.empty());
  ASSERT_TRUE(controller_->params_.state_interfaces.empty());
  ASSERT_TRUE(controller_->params_.name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.command_interfaces, testing::ElementsAreArray(command_interface_names_));
  ASSERT_THAT(controller_->params_.state_interfaces, testing::ElementsAreArray(state_interface_names_));
  ASSERT_EQ(controller_->params_.name, interface_namespace_);
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
