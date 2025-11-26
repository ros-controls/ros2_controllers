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
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].u_clamp_max, 5.0);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].u_clamp_min, -5.0);
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
  size_t expected_ref_size = dof_names_.size() * state_interfaces_.size() +
                             (controller_->params_.export_params.gain_references
                                ? dof_names_.size() * controller_->GAIN_INTERFACES.size()
                                : 0);
  ASSERT_EQ(ref_if_conf.size(), expected_ref_size);
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
  auto msg = controller_->input_ref_.get();
  EXPECT_EQ(msg.values.size(), dof_names_.size());
  for (const auto & cmd : msg.values)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }
  EXPECT_EQ(msg.values_dot.size(), dof_names_.size());
  for (const auto & cmd : msg.values_dot)
  {
    EXPECT_TRUE(std::isnan(cmd));
  }
  size_t expected_ref_size = dof_names_.size() * state_interfaces_.size() +
                             (controller_->params_.export_params.gain_references
                                ? dof_names_.size() * controller_->GAIN_INTERFACES.size()
                                : 0);
  EXPECT_EQ(controller_->reference_interfaces_.size(), expected_ref_size);
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
  ASSERT_EQ(controller_->command_interfaces_[0].get_optional().value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  ASSERT_TRUE(std::isnan(controller_->measured_state_values_[0]));
  ASSERT_EQ(controller_->command_interfaces_[0].get_optional().value(), 101.101);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->reference_interfaces_[0]));
  ASSERT_TRUE(std::isnan(controller_->measured_state_values_[0]));
  ASSERT_EQ(controller_->command_interfaces_[0].get_optional().value(), 101.101);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

/**
 * @brief Check the update logic in non chained mode with feedforward gain is 0
 *
 */

TEST_F(PidControllerTest, test_update_logic_zero_feedforward_gain)
{
  SetUpController("test_pid_controller_unlimited");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());
  EXPECT_TRUE(std::isnan(controller_->input_ref_.get().values[0]));
  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  controller_->set_reference(dof_command_values_);

  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_FALSE(std::isnan(controller_->input_ref_.get().values[i]));
    EXPECT_EQ(controller_->input_ref_.get().values[i], dof_command_values_[i]);
    EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
  }

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  size_t expected_ref_size = dof_names_.size() * state_interfaces_.size() +
                             (controller_->params_.export_params.gain_references
                                ? dof_names_.size() * controller_->GAIN_INTERFACES.size()
                                : 0);
  EXPECT_EQ(controller_->reference_interfaces_.size(), expected_ref_size);
  EXPECT_EQ(controller_->reference_interfaces_.size(), expected_ref_size);
  for (size_t i = 0; i < dof_command_values_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(controller_->input_ref_.get().values[i]));

    // check the command value:
    // ref = 101.101, state = 1.1, ds = 0.01
    // error = ref - state = 100.001, error_dot = error/ds = 10000.1,
    // p_term = 100.001 * 1, i_term = 0.0 at first update call, d_term = error/ds = 10000.1 * 3
    // feedforward ON, feedforward_gain = 0
    // -> cmd = p_term + i_term + d_term + feedforward_gain * ref = 30100.3 + 0 * 101.101 = 30102.3
    const double expected_command_value = 30100.301000;

    double actual_value =
      std::round(controller_->command_interfaces_[0].get_optional().value() * 1e5) / 1e5;
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
  SetUpController("test_pid_controller_unlimited");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  // set chain mode to true
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(true);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());
  // feedforward mode is off as default, use this for convenience

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
  size_t expected_ref_size = dof_names_.size() * state_interfaces_.size() +
                             (controller_->params_.export_params.gain_references
                                ? dof_names_.size() * controller_->GAIN_INTERFACES.size()
                                : 0);

  EXPECT_EQ(controller_->reference_interfaces_.size(), expected_ref_size);
  EXPECT_EQ(controller_->reference_interfaces_.size(), expected_ref_size);

  // check the command value
  // ref = 5.0, state = 1.1, ds = 0.01, p_gain = 1.0, i_gain = 2.0, d_gain = 3.0
  // error = ref - state =  5.0 - 1.1 = 3.9, error_dot = error/ds = 3.9/0.01 = 390.0,
  // p_term = error * p_gain = 3.9 * 1.0 = 3.9,
  // i_term = zero at first update
  // d_term = error_dot * d_gain = 390.0 * 3.0 = 1170.0
  // feedforward OFF -> cmd = p_term + i_term + d_term = 3.9 + 0.078 + 1170.0 = 1173.978
  {
    const double expected_command_value = 1173.9;
    EXPECT_EQ(controller_->command_interfaces_[0].get_optional().value(), expected_command_value);
  }
}

/**
 * @brief check default calculation with angle_wraparound turned off
 */
TEST_F(PidControllerTest, test_update_logic_angle_wraparound_off)
{
  SetUpController("test_pid_controller_unlimited");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->params_.gains.dof_names_map[dof_names_[0]].angle_wraparound);

  // write reference interface so that the values would be wrapped
  controller_->reference_interfaces_[0] = 10.0;

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check the result of the commands - the values are not wrapped
  // ref = 10.0, state = 1.1, ds = 0.01, p_gain = 1.0, i_gain = 2.0, d_gain = 3.0
  // error = ref - state =  10.0 - 1.1 = 8.9, error_dot = error/ds = 8.9/0.01 = 890.0,
  // p_term = error * p_gain = 8.9 * 1.0 = 8.9,
  // i_term = zero at first update
  // d_term = error_dot * d_gain = 890.0 * 3.0 = 2670.0
  // feedforward OFF -> cmd = p_term + i_term + d_term = 8.9 + 0.0 + 2670.0 = 2678.9
  const double expected_command_value = 2678.9;
  EXPECT_NEAR(
    controller_->command_interfaces_[0].get_optional().value(), expected_command_value, 1e-5);
}

/**
 * @brief check default calculation with angle_wraparound turned off
 */
TEST_F(PidControllerTest, test_update_logic_angle_wraparound_on)
{
  SetUpController("test_pid_controller_angle_wraparound_on");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

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

  // Check the command value with wrapped error
  // ref = 10.0, state = 1.1, ds = 0.01, p_gain = 1.0, i_gain = 2.0, d_gain = 3.0
  // error = ref - state =  wrap(10.0 - 1.1) = 8.9-2*pi = 2.616814, error_dot = error/ds
  // = 2.6168/0.01 = 261.6814, p_term = error * p_gain = 2.6168 * 1.0 = 2.6168, i_term = zero at
  // first update d_term = error_dot * d_gain = 261.6814 * 3.0 = 785.0444079 feedforward OFF -> cmd
  // = p_term + i_term + d_term = 2.616814, + 0.0 + 785.0444079 = 787.6612219
  const double expected_command_value = 787.6612219;
  EXPECT_NEAR(
    controller_->command_interfaces_[0].get_optional().value(), expected_command_value, 1e-5);
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

  size_t num_control_references = dof_names_.size() * state_interfaces_.size();

  for (size_t i = 0; i < num_control_references; ++i)
  {
    ASSERT_EQ(controller_->reference_interfaces_[i], 0.45);
  }
  if (controller_->params_.export_params.gain_references)
  {
    for (size_t i = num_control_references; i < controller_->reference_interfaces_.size(); ++i)
    {
      // Check the remaining interfaces (P, I, D gains) are NaN
      EXPECT_TRUE(std::isnan(controller_->reference_interfaces_[i]));
    }
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
 * @brief check chained pid controller with feedforward gain as non-zero, single interface
 */
TEST_F(PidControllerTest, test_update_chained_non_zero_feedforward_gain)
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

  controller_->set_chained_mode(true);

  // activate controller
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // send a message to update reference interface
  controller_->set_reference({target_value});
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check on result from update
  ASSERT_EQ(controller_->command_interfaces_[0].get_optional().value(), expected_command_value);
}

/**
 * @brief check chained pid controller with feedforward gain changed during runtime
 */
TEST_F(PidControllerTest, test_update_chained_changing_feedforward_gain)
{
  // state interface value is 1.1 as defined in test fixture
  // given target value 5.0
  // with p gain 0.5, the command value should be 0.5 * (5.0 - 1.1) = 1.95
  // with feedforward gain 1.0, the command value should be 1.95 + 1.0 * 5.0 = 6.95

  constexpr double target_value = 5.0;
  constexpr double first_expected_command_value = 1.95;
  constexpr double second_expected_command_value = 6.95;

  SetUpController("test_pid_controller_with_feedforward_gain");

  for (const auto & dof_name : dof_names_)
  {
    ASSERT_TRUE(controller_->get_node()
                  ->set_parameter(rclcpp::Parameter("gains." + dof_name + ".feedforward_gain", 0.0))
                  .successful);
  }

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check on interfaces & pid gain parameters
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 0.5);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 0.0);
  }
  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  EXPECT_THAT(
    controller_->params_.reference_and_state_interfaces,
    testing::ElementsAreArray(state_interfaces_));
  ASSERT_FALSE(controller_->params_.use_external_measured_states);

  // setup executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  controller_->set_chained_mode(true);

  // activate controller
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // send a message to update reference interface
  controller_->set_reference({target_value});
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check on result from update
  ASSERT_EQ(
    controller_->command_interfaces_[0].get_optional().value(), first_expected_command_value);

  // turn on feedforward gain
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_TRUE(controller_->get_node()
                  ->set_parameter(rclcpp::Parameter("gains." + dof_name + ".feedforward_gain", 1.0))
                  .successful);
  }

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check on result from update
  ASSERT_EQ(
    controller_->command_interfaces_[0].get_optional().value(), second_expected_command_value);

  // Check updated parameters
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 1.0);
  }
}

/**
 * @brief Test if retention of the integral state is deactivated
 *
 */

TEST_F(PidControllerTest, test_save_i_term_off)
{
  SetUpController("test_save_i_term_off");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_chained_mode(false);
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_FALSE(controller_->params_.gains.dof_names_map[dof_name].save_i_term);
  }
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  controller_->set_reference(dof_command_values_);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check the command value
  // error = ref - state = 100.001, error_dot = error/ds = 10000.1,
  // p_term = 100.001 * 1, i_term = zero at first update, d_term = error/ds = 10000.1 * 3
  // feedforward OFF -> cmd = p_term + i_term + d_term = 30100.301
  const double expected_command_value = 30100.3010;

  double actual_value =
    std::round(controller_->command_interfaces_[0].get_optional().value() * 1e5) / 1e5;
  EXPECT_NEAR(actual_value, expected_command_value, 1e-5);

  // deactivate the controller and set command=state
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_reference(dof_state_values_);

  // reactivate the controller, the integral term should NOT be saved
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  actual_value = std::round(controller_->command_interfaces_[0].get_optional().value() * 1e5) / 1e5;
  EXPECT_NEAR(actual_value, 0.0, 1e-5);
}

/**
 * @brief Test if retention of the integral state is working
 *
 */

TEST_F(PidControllerTest, test_save_i_term_on)
{
  SetUpController("test_save_i_term_on");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_TRUE(controller_->params_.gains.dof_names_map[dof_name].save_i_term);
  }
  controller_->set_chained_mode(false);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  controller_->set_reference(dof_command_values_);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check the command value
  // error = ref - state = 100.001, error_dot = error/ds = 10000.1,
  // p_term = 100.001 * 1, i_term = zero at first update, d_term = error/ds = 10000.1 * 3
  // feedforward OFF -> cmd = p_term + i_term + d_term = 30102.301
  const double expected_command_value = 30100.3010;

  double actual_value =
    std::round(controller_->command_interfaces_[0].get_optional().value() * 1e5) / 1e5;
  EXPECT_NEAR(actual_value, expected_command_value, 1e-5);

  // deactivate the controller and set command=state
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  controller_->set_reference(dof_state_values_);

  // reactivate the controller, the integral term should be saved
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  actual_value = std::round(controller_->command_interfaces_[0].get_optional().value() * 1e5) / 1e5;
  EXPECT_NEAR(actual_value, 2.00002, 1e-5);  // i_term from above
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
