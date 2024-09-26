// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Daniel Azanov, Dr. Denis
//

#include "test_pid_controller.hpp"

#include <memory>
#include <string>
#include <vector>

using pid_controller::feedforward_mode_type;

class PidControllerTest : public PidControllerFixture<TestablePidController>
{
};

TEST_F(PidControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.dof_names.empty());
  ASSERT_TRUE(controller_->params_.reference_and_state_dof_names.empty());
  ASSERT_TRUE(controller_->params_.command_interface.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.dof_names, testing::ElementsAreArray(dof_names_));
  ASSERT_THAT(
    controller_->params_.reference_and_state_dof_names,
    testing::ElementsAreArray(reference_and_state_dof_names_));
  ASSERT_THAT(
    controller_->reference_and_state_dof_names_,
    testing::ElementsAreArray(reference_and_state_dof_names_));
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
}

TEST_F(PidControllerTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), dof_command_values_.size());
  for (size_t i = 0; i < cmd_if_conf.names.size(); ++i)
  {
    EXPECT_EQ(cmd_if_conf.names[i], dof_names_[i] + "/" + command_interface_);
  }
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), dof_state_values_.size());
  size_t si_index = 0;
  for (const auto & interface : state_interfaces_)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      EXPECT_EQ(state_if_conf.names[si_index], dof_name + "/" + interface);
      ++si_index;
    }
  }
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // check ref itfs
  auto ref_if_conf = controller_->export_reference_interfaces();
  ASSERT_EQ(ref_if_conf.size(), dof_state_values_.size());
  size_t ri_index = 0;
  for (const auto & interface : state_interfaces_)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      const std::string ref_itf_name =
        std::string(controller_->get_node()->get_name()) + "/" + dof_name + "/" + interface;
      EXPECT_EQ(ref_if_conf[ri_index]->get_name(), ref_itf_name);
      EXPECT_EQ(ref_if_conf[ri_index]->get_prefix_name(), controller_->get_node()->get_name());
      EXPECT_EQ(ref_if_conf[ri_index]->get_interface_name(), dof_name + "/" + interface);
      ++ri_index;
    }
  }

  // check exported state itfs
  auto exported_state_itfs = controller_->export_state_interfaces();
  ASSERT_EQ(exported_state_itfs.size(), dof_state_values_.size());
  size_t esi_index = 0;
  for (const auto & interface : state_interfaces_)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      const std::string state_itf_name =
        std::string(controller_->get_node()->get_name()) + "/" + dof_name + "/" + interface;
      EXPECT_EQ(exported_state_itfs[esi_index].get_name(), state_itf_name);
      EXPECT_EQ(
        exported_state_itfs[esi_index].get_prefix_name(), controller_->get_node()->get_name());
      EXPECT_EQ(exported_state_itfs[esi_index].get_interface_name(), dof_name + "/" + interface);
      ++esi_index;
    }
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
