// Copyright 2022 VoodooIT, sole proprietorship
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

#include "test_twist_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// When there are many mandatory parameters, set all by default and remove one by one in a
// parameterized test
TEST_P(TwistControllerTestParameterizedParameters, one_parameter_is_missing)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

// TODO(anyone): the new gtest version after 1.8.0 uses INSTANTIATE_TEST_SUITE_P
INSTANTIATE_TEST_SUITE_P(
  MissingMandatoryParameterDuringConfiguration, TwistControllerTestParameterizedParameters,
  ::testing::Values(
    std::make_tuple(std::string("joint"), rclcpp::ParameterValue(std::string())),
    std::make_tuple(
      std::string("interface_names"), rclcpp::ParameterValue(std::vector<std::string>({})))));

TEST_F(TwistControllerTest, joint_names_parameter_not_set)
{
  SetUpController(false);

  ASSERT_TRUE(controller_->joint_name_.empty());
  ASSERT_TRUE(controller_->interface_names_.empty());

  controller_->get_node()->set_parameter({"interface_names", interface_names_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_TRUE(controller_->joint_name_.empty());
  ASSERT_TRUE(controller_->interface_names_.empty());
}

TEST_F(TwistControllerTest, interface_parameter_not_set)
{
  SetUpController(false);

  controller_->get_node()->set_parameter({"joint", joint_name_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_TRUE(!controller_->joint_name_.empty());
  ASSERT_TRUE(controller_->interface_names_.empty());
}

TEST_F(TwistControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->joint_name_.empty());
  ASSERT_TRUE(controller_->interface_names_.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(!controller_->joint_name_.empty());
  ASSERT_TRUE(controller_->joint_name_.size() == joint_name_.size());
  ASSERT_TRUE(controller_->joint_name_ == joint_name_);

  ASSERT_TRUE(!controller_->interface_names_.empty());
  ASSERT_TRUE(std::equal(
    controller_->interface_names_.begin(), controller_->interface_names_.end(),
    interface_names_.begin(), interface_names_.end()));
}

TEST_F(TwistControllerTest, check_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
}

TEST_F(TwistControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(TwistControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  rclcpp::Time node_time = controller_->get_node()->now();
  rclcpp::Duration duration = rclcpp::Duration::from_nanoseconds(1000000);  // 1ms
  ASSERT_EQ(controller_->update(node_time, duration), controller_interface::return_type::OK);
}

TEST_F(TwistControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(TwistControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  rclcpp::Time node_time = controller_->get_node()->now();
  rclcpp::Duration duration = rclcpp::Duration::from_nanoseconds(1000000);  // 1ms
  ASSERT_EQ(controller_->update(node_time, duration), controller_interface::return_type::OK);
}

TEST_F(TwistControllerTest, command_callback_test)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  // default values
  ASSERT_EQ(command_itfs_.size(), 6u);

  ASSERT_EQ(command_itfs_[0].get_value(), 0.0);
  ASSERT_EQ(command_itfs_[1].get_value(), 0.0);
  ASSERT_EQ(command_itfs_[2].get_value(), 0.1);
  ASSERT_EQ(command_itfs_[3].get_value(), 0.0);
  ASSERT_EQ(command_itfs_[4].get_value(), 0.0);
  ASSERT_EQ(command_itfs_[5].get_value(), 0.0);

  auto node_state = controller_->get_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // send a new command and wait
  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  // update successful
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(command_itfs_[0].get_value(), 10.0);
  ASSERT_EQ(command_itfs_[1].get_value(), 10.0);
  ASSERT_EQ(command_itfs_[2].get_value(), 10.0);
  ASSERT_EQ(command_itfs_[3].get_value(), 0.1);
  ASSERT_EQ(command_itfs_[4].get_value(), 0.1);
  ASSERT_EQ(command_itfs_[5].get_value(), 0.1);
}
