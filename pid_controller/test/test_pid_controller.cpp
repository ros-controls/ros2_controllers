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

#include <limits>
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
  ASSERT_TRUE(controller_->params_.reference_and_state_interfaces.empty());
  ASSERT_FALSE(controller_->params_.use_external_measured_states);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.dof_names, testing::ElementsAreArray(dof_names_));
  ASSERT_TRUE(controller_->params_.reference_and_state_dof_names.empty());
  ASSERT_THAT(controller_->reference_and_state_dof_names_, testing::ElementsAreArray(dof_names_));
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 1.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i, 2.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].d, 3.0);
    ASSERT_FALSE(controller_->params_.gains.dof_names_map[dof_name].antiwindup);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i_clamp_max, 5.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i_clamp_min, -5.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 0.0);
  }
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  EXPECT_THAT(
    controller_->params_.reference_and_state_interfaces,
    testing::ElementsAreArray(state_interfaces_));
  ASSERT_FALSE(controller_->params_.use_external_measured_states);
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
    for (const auto & dof_name : dof_names_)
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
    for (const auto & dof_name : dof_names_)
    {
      const std::string ref_itf_name =
        std::string(controller_->get_node()->get_name()) + "/" + dof_name + "/" + interface;
      EXPECT_EQ(ref_if_conf[ri_index]->get_name(), ref_itf_name);
      EXPECT_EQ(
        ref_if_conf[ri_index]->get_prefix_name(),
        std::string(controller_->get_node()->get_name()) + "/" + dof_name);
      EXPECT_EQ(ref_if_conf[ri_index]->get_interface_name(), interface);
      ++ri_index;
    }
  }

  // check exported state interfaces
  auto state_ifs = controller_->export_state_interfaces();
  ASSERT_EQ(state_ifs.size(), dof_state_values_.size());
  size_t s_index = 0;
  for (const auto & interface : state_interfaces_)
  {
    for (const auto & dof_name : dof_names_)
    {
      const std::string state_itf_name =
        std::string(controller_->get_node()->get_name()) + "/" + dof_name + "/" + interface;
      EXPECT_EQ(state_ifs[s_index]->get_name(), state_itf_name);
      EXPECT_EQ(
        state_ifs[s_index]->get_prefix_name(),
        std::string(controller_->get_node()->get_name()) + "/" + dof_name);
      EXPECT_EQ(state_ifs[s_index]->get_interface_name(), interface);
      ++s_index;
    }
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

  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_state_values_.size());
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
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  ASSERT_TRUE(std::isnan(controller_->measured_state_values_[0]));
  ASSERT_EQ(controller_->command_interfaces_[0].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  ASSERT_TRUE(std::isnan(controller_->measured_state_values_[0]));
  ASSERT_EQ(controller_->command_interfaces_[0].get_value(), 101.101);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  ASSERT_TRUE(std::isnan(controller_->measured_state_values_[0]));
  ASSERT_EQ(controller_->command_interfaces_[0].get_value(), 101.101);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(PidControllerTest, test_feedforward_mode_service)
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

/**
 * @brief Check the update logic in non chained mode with feedforward OFF
 *
 */

TEST_F(PidControllerTest, test_update_logic_feedforward_off)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[0]));
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), 0.0);
  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    msg->values[i] = dof_command_values_[i];
  }
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);

  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[i]));
    EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->values[i], dof_command_values_[i]);
    EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
  EXPECT_EQ(
    controller_->reference_interfaces_.size(), dof_names_.size() * state_interfaces_.size());
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_state_values_.size());
  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[i]));
  }
  // check the command value
  // error = ref - state = 100.001, error_dot = error/ds = 10000.1,
  // p_term = 100.001 * 1, i_term = 1.00001 * 2 = 2.00002, d_term = error/ds = 10000.1 * 3
  // feedforward OFF -> cmd = p_term + i_term + d_term = 30102.3
  const double expected_command_value = 30102.30102;

  double actual_value = std::round(controller_->command_interfaces_[0].get_value() * 1e5) / 1e5;
  EXPECT_NEAR(actual_value, expected_command_value, 1e-5);
}

/**
 * @brief Check the update logic in non chained mode with feedforward ON and feedforward gain is 0
 *
 */

TEST_F(PidControllerTest, test_update_logic_feedforward_on_with_zero_feedforward_gain)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());
  EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[0]));
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = dof_names_;
  msg->values.resize(dof_names_.size(), 0.0);
  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    msg->values[i] = dof_command_values_[i];
  }
  msg->values_dot.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);

  controller_->control_mode_.writeFromNonRT(feedforward_mode_type::ON);
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);

  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_FALSE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[i]));
    EXPECT_EQ((*(controller_->input_ref_.readFromRT()))->values[i], dof_command_values_[i]);
    EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);
  EXPECT_EQ(
    controller_->reference_interfaces_.size(), dof_names_.size() * state_interfaces_.size());
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_state_values_.size());
  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan((*(controller_->input_ref_.readFromRT()))->values[i]));

    // check the command value:
    // ref = 101.101, state = 1.1, ds = 0.01
    // error = ref - state = 100.001, error_dot = error/ds = 10000.1,
    // p_term = 100.001 * 1, i_term = 1.00001 * 2 = 2.00002, d_term = error/ds = 10000.1 * 3
    // feedforward ON, feedforward_gain = 0
    // -> cmd = p_term + i_term + d_term + feedforward_gain * ref = 30102.3 + 0 * 101.101 = 30102.3
    const double expected_command_value = 30102.301020;

    double actual_value = std::round(controller_->command_interfaces_[0].get_value() * 1e5) / 1e5;
    EXPECT_NEAR(actual_value, expected_command_value, 1e-5);
  }
}

/**
 * @brief Check the update logic when chain mode is on.
 *  in chain mode, update_reference_from_subscribers is not called from update method, and the
 * reference value is used for calculation
 */

TEST_F(PidControllerTest, test_update_logic_chainable_not_use_subscriber_update)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  // set chain mode to true
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());
  // feedforward mode is off as default, use this for convenience
  EXPECT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);

  // update reference interface which will be used for calculation
  const double ref_interface_value = 5.0;
  controller_->reference_interfaces_[0] = ref_interface_value;

  // publish a command message which should be ignored as chain mode is on
  publish_commands({10.0}, {0.0});
  controller_->wait_for_commands(executor);

  // check the reference interface is not updated as chain mode is on
  EXPECT_EQ(controller_->reference_interfaces_[0], ref_interface_value);

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_TRUE(controller_->is_in_chained_mode());

  EXPECT_EQ(
    controller_->reference_interfaces_.size(), dof_names_.size() * state_interfaces_.size());
  EXPECT_EQ(controller_->reference_interfaces_.size(), dof_state_values_.size());

  // check the command value
  // ref = 5.0, state = 1.1, ds = 0.01, p_gain = 1.0, i_gain = 2.0, d_gain = 3.0
  // error = ref - state =  5.0 - 1.1 = 3.9, error_dot = error/ds = 3.9/0.01 = 390.0,
  // p_term = error * p_gain = 3.9 * 1.0 = 3.9,
  // i_term = error * ds * i_gain = 3.9 * 0.01 * 2.0 = 0.078,
  // d_term = error_dot * d_gain = 390.0 * 3.0 = 1170.0
  // feedforward OFF -> cmd = p_term + i_term + d_term = 3.9 + 0.078 + 1170.0 = 1173.978
  const double expected_command_value = 1173.978;

  EXPECT_EQ(controller_->command_interfaces_[0].get_value(), expected_command_value);
}

/**
 * @brief check default calculation with angle_wraparound turned off
 */
TEST_F(PidControllerTest, test_update_logic_angle_wraparound_off)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->params_.gains.dof_names_map[dof_names_[0]].angle_wraparound);

  // write reference interface so that the values are would be wrapped
  controller_->reference_interfaces_[0] = 10.0;

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check the result of the commands - the values are not wrapped
  const double expected_command_value = 2679.078;
  EXPECT_NEAR(controller_->command_interfaces_[0].get_value(), expected_command_value, 1e-5);
}

/**
 * @brief check default calculation with angle_wraparound turned off
 */
TEST_F(PidControllerTest, test_update_logic_angle_wraparound_on)
{
  SetUpController("test_pid_controller_angle_wraparound_on");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // Check on wraparound is on
  ASSERT_TRUE(controller_->params_.gains.dof_names_map[dof_names_[0]].angle_wraparound);

  // Write reference interface with values that would wrap, state is 1.1
  controller_->reference_interfaces_[0] = 10.0;

  // Run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // Check the command value
  const double expected_command_value = 787.713559;
  EXPECT_NEAR(controller_->command_interfaces_[0].get_value(), expected_command_value, 1e-5);
}

TEST_F(PidControllerTest, subscribe_and_get_messages_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.dof_states.size(), dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    ASSERT_EQ(msg.dof_states[i].name, dof_names_[i]);
    EXPECT_TRUE(std::isnan(msg.dof_states[i].reference));
    ASSERT_EQ(msg.dof_states[i].output, dof_command_values_[i]);
  }
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

  ASSERT_EQ(msg.dof_states.size(), dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    ASSERT_EQ(msg.dof_states[i].name, dof_names_[i]);
    EXPECT_TRUE(std::isnan(msg.dof_states[i].reference));
    ASSERT_EQ(msg.dof_states[i].output, dof_command_values_[i]);
  }

  for (size_t i = 0; i < controller_->reference_interfaces_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
  }

  publish_commands();
  controller_->wait_for_commands(executor);

  for (size_t i = 0; i < controller_->reference_interfaces_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (size_t i = 0; i < controller_->reference_interfaces_.size(); ++i)
  {
    ASSERT_EQ(controller_->reference_interfaces_[i], 0.45);
  }

  subscribe_and_get_messages(msg);

  ASSERT_EQ(msg.dof_states.size(), dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    ASSERT_EQ(msg.dof_states[i].name, dof_names_[i]);
    ASSERT_EQ(msg.dof_states[i].reference, 0.45);
    ASSERT_NE(msg.dof_states[i].output, dof_command_values_[i]);
  }
}

/**
 * @brief check chained pid controller with feedforward and gain as non-zero, single interface
 */
TEST_F(PidControllerTest, test_update_chained_feedforward_with_gain)
{
  // state interface value is 1.1 as defined in test fixture
  // with p gain 0.5, the command value should be 0.5 * (5.0 - 1.1) = 1.95
  // with feedforward gain 1.0, the command value should be 1.95 + 1.0 * 5.0 = 6.95
  const double target_value = 5.0;
  const double expected_command_value = 6.95;

  SetUpController("test_pid_controller_with_feedforward_gain");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check on interfaces & pid gain parameters
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 0.5);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 1.0);
  }
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  EXPECT_THAT(
    controller_->params_.reference_and_state_interfaces,
    testing::ElementsAreArray(state_interfaces_));
  ASSERT_FALSE(controller_->params_.use_external_measured_states);

  // setup executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  controller_->set_chained_mode(true);

  // activate controller
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // turn on feedforward
  controller_->control_mode_.writeFromNonRT(feedforward_mode_type::ON);
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::ON);

  // send a message to update reference interface
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = controller_->params_.dof_names;
  msg->values.resize(msg->dof_names.size(), 0.0);
  for (size_t i = 0; i < msg->dof_names.size(); ++i)
  {
    msg->values[i] = target_value;
  }
  msg->values_dot.resize(msg->dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check on result from update
  ASSERT_EQ(controller_->command_interfaces_[0].get_value(), expected_command_value);
}

/**
 * @brief check chained pid controller with feedforward OFF and gain as non-zero, single interface
 */
TEST_F(PidControllerTest, test_update_chained_feedforward_off_with_gain)
{
  // state interface value is 1.1 as defined in test fixture
  // given target value 5.0
  // with p gain 0.5, the command value should be 0.5 * (5.0 - 1.1) = 1.95
  // with feedforward off, the command value should be still 1.95 even though feedforward gain
  // is 1.0
  const double target_value = 5.0;
  const double expected_command_value = 1.95;

  SetUpController("test_pid_controller_with_feedforward_gain");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check on interfaces & pid gain parameters
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 0.5);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 1.0);
  }
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  EXPECT_THAT(
    controller_->params_.reference_and_state_interfaces,
    testing::ElementsAreArray(state_interfaces_));
  ASSERT_FALSE(controller_->params_.use_external_measured_states);

  // setup executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  controller_->set_chained_mode(true);

  // activate controller
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // feedforward by default is OFF
  ASSERT_EQ(*(controller_->control_mode_.readFromRT()), feedforward_mode_type::OFF);

  // send a message to update reference interface
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->dof_names = controller_->params_.dof_names;
  msg->values.resize(msg->dof_names.size(), 0.0);
  for (size_t i = 0; i < msg->dof_names.size(); ++i)
  {
    msg->values[i] = target_value;
  }
  msg->values_dot.resize(msg->dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  controller_->input_ref_.writeFromNonRT(msg);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check on result from update
  ASSERT_EQ(controller_->command_interfaces_[0].get_value(), expected_command_value);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
