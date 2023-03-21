// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_pid_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using pid_controller::CMD_MY_ITFS;
using pid_controller::feedforward_mode_type;
using pid_controller::STATE_MY_ITFS;

class PidControllerTest : public PidControllerFixture<TestablePidController>
{
};

TEST_F(PidControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.dof_names.empty());
  ASSERT_TRUE(controller_->params_.reference_and_state_dof_names.empty());
  ASSERT_TRUE(controller_->params_.command_interface.empty());
  ASSERT_TRUE(controller_->params_.reference_and_state_interfaces.empty());
  ASSERT_FALSE(controller_->params_.use_external_measured_states);
  ASSERT_FALSE(controller_->params_.runtime_param_update);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.dof_names, testing::ElementsAreArray(dof_names_));
  ASSERT_TRUE(controller_->params_.reference_and_state_dof_names.empty());
  ASSERT_THAT(controller_->reference_and_state_dof_names_, testing::ElementsAreArray(dof_names_));
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 1.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i, 2.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].d, 10.0);
    ASSERT_FALSE(controller_->params_.gains.dof_names_map[dof_name].antiwindup);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i_clamp_max, 5.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i_clamp_min, -5.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 0.0);
  }
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  ASSERT_TRUE(controller_->params_.reference_and_state_interfaces.empty());
  ASSERT_FALSE(controller_->params_.use_external_measured_states);
  ASSERT_FALSE(controller_->params_.runtime_param_update);
}

TEST_F(PidControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), dof_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], dof_names_[i] + "/" + command_interface_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), dof_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], dof_names_[i] + "/" + command_interface_);
  }

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    const std::string ref_itf_name = std::string(controller_->get_node()->get_name()) + "/" +
                                     dof_names_[i] + "/" + command_interface_;
    EXPECT_EQ(reference_interfaces[i].get_name(), ref_itf_name);
    EXPECT_EQ(reference_interfaces[i].get_prefix_name(), controller_->get_node()->get_name());
    EXPECT_EQ(
      reference_interfaces[i].get_interface_name(), dof_names_[i] + "/" + command_interface_);
  }
}

TEST_F(PidControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_ref_.readFromNonRT();
  EXPECT_EQ((*msg)->values.size(), dof_names_.size());
  for (const auto & cmd : (*msg)->values)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ((*msg)->values_dot.size(), dof_names_.size());
  for (const auto & cmd : (*msg)->values_dot)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }

  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(PidControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(PidControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(PidControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[CMD_MY_ITFS].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CMD_MY_ITFS].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CMD_MY_ITFS].get_value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(PidControllerTest, test_setting_slow_mode_service)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  // initially set to OFF
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // should stay false
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);

  // set to true
  ASSERT_NO_THROW(call_service(true, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);

  // set back to false
  ASSERT_NO_THROW(call_service(false, executor));
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
}

TEST_F(PidControllerTest, test_update_logic_fast)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), TEST_DISPLACEMENT);
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);
  EXPECT_EQ(dof_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[0]));
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(PidControllerTest, test_update_logic_slow)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), TEST_DISPLACEMENT);
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(feedforward_mode_type::OFF);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->values[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(dof_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT / 2);
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[0]));
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(PidControllerTest, test_update_logic_chainable_fast)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), TEST_DISPLACEMENT);
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);
  // this is input source in chained mode
  controller_->reference_interfaces_[STATE_MY_ITFS] = TEST_DISPLACEMENT * 2;

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);
  // reference_interfaces is directly applied
  EXPECT_EQ(dof_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->values[0], TEST_DISPLACEMENT);
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(PidControllerTest, test_update_logic_chainable_slow)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // set command statically
  static constexpr double TEST_DISPLACEMENT = 23.24;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  // When slow mode is enabled command ends up being half of the value
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), TEST_DISPLACEMENT);
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);
  controller_->control_mode_.writeFromNonRT(feedforward_mode_type::OFF);
  // this is input source in chained mode
  controller_->reference_interfaces_[STATE_MY_ITFS] = TEST_DISPLACEMENT * 4;

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
  ASSERT_EQ((*(controller_->input_ref_.readFromRT()))->values[0], TEST_DISPLACEMENT);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // reference_interfaces is directly applied
  EXPECT_EQ(dof_command_values_[STATE_MY_ITFS], TEST_DISPLACEMENT * 2);
  // message is not touched in chained mode
  EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->values[0], TEST_DISPLACEMENT);
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_names_.size());
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
}

TEST_F(PidControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.dof_states.size(), 1);
  ASSERT_EQ(msg.dof_states[0].set_point, 101.101);
}

TEST_F(PidControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.dof_states.size(), 1);
  ASSERT_EQ(msg.dof_states[0].set_point, 101.101);

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(dof_command_values_[CMD_MY_ITFS], 0.45);

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.dof_states.size(), 1);
  ASSERT_EQ(msg.dof_states[0].set_point, 0.45);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
