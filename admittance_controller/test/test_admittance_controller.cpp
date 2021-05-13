// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
/// \author: Denis Stogl

#include "test_admittance_controller.hpp"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

// When there are many mandatory parameters, set all by default and remove one by one in a
// parameterized test
TEST_P(AdmittanceControllerTestParameterizedParameters, one_parameter_is_missing)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

// TODO(anyone): the new gtest version afer 1.8.0 uses INSTANTIATE_TEST_SUITE_P
INSTANTIATE_TEST_CASE_P(
  MissingMandatoryParameterDuringConfiguration,
  AdmittanceControllerTestParameterizedParameters,
  ::testing::Values(
    std::make_tuple(
      std::string("joints"),
      rclcpp::ParameterValue(std::vector<std::string>())
    ),
    std::make_tuple(
      std::string("command_interfaces"),
      rclcpp::ParameterValue(std::vector<std::string>())
    ),
    std::make_tuple(
      std::string("state_interfaces"),
      rclcpp::ParameterValue(std::vector<std::string>())
    ),
    std::make_tuple(
      std::string("ft_sensor_name"),
      rclcpp::ParameterValue("")
    ),
//     std::make_tuple(
//       std::string("use_joint_commands_as_input"),
//       rclcpp::ParameterValue(false)
//     ),
    std::make_tuple(
      std::string("IK.base"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("IK.tip"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("IK.group_name"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("control_frame"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("endeffector_frame"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("fixed_world_frame"),
      rclcpp::ParameterValue("")
    ),
    std::make_tuple(
      std::string("sensor_frame"),
      rclcpp::ParameterValue("")
    ),
    // TODO(anyone): this tests are unstable...
//     std::make_tuple(
//       std::string("admittance.selected_axes.x"),
//       rclcpp::ParameterValue(false)
//     ),
//     std::make_tuple(
//       std::string("admittance.selected_axes.y"),
//       rclcpp::ParameterValue(false)
//     ),
//     std::make_tuple(
//       std::string("admittance.selected_axes.z"),
//       rclcpp::ParameterValue(false)
//     ),
//     std::make_tuple(
//       std::string("admittance.selected_axes.rx"),
//       rclcpp::ParameterValue(false)
//     ),
//     std::make_tuple(
//       std::string("admittance.selected_axes.ry"),
//       rclcpp::ParameterValue(false)
//     ),
//     std::make_tuple(
//       std::string("admittance.selected_axes.rz"),
//       rclcpp::ParameterValue(false)
//     ),
    std::make_tuple(
      std::string("admittance.mass.x"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.mass.y"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.mass.z"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.mass.rx"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.mass.ry"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.mass.rz"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
        std::make_tuple(
      std::string("admittance.damping.x"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.damping.y"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.damping.z"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.damping.rx"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.damping.ry"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.damping.rz"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
        std::make_tuple(
      std::string("admittance.stiffness.x"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.stiffness.y"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.stiffness.z"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.stiffness.rx"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.stiffness.ry"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    ),
    std::make_tuple(
      std::string("admittance.stiffness.rz"),
      rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN())
    )
  )
);

TEST_F(AdmittanceControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->command_interface_types_.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(!controller_->joint_names_.empty());
  ASSERT_TRUE(controller_->joint_names_.size() == joint_names_.size());
  ASSERT_TRUE(std::equal(controller_->joint_names_.begin(), controller_->joint_names_.end(),
    joint_names_.begin(), joint_names_.end()));

  ASSERT_TRUE(!controller_->command_interface_types_.empty());
  ASSERT_TRUE(controller_->command_interface_types_.size() == command_interface_types_.size());
  ASSERT_TRUE(std::equal(
    controller_->command_interface_types_.begin(), controller_->command_interface_types_.end(),
    command_interface_types_.begin(), command_interface_types_.end()));

  ASSERT_TRUE(!controller_->state_interface_types_.empty());
  ASSERT_TRUE(controller_->state_interface_types_.size() == state_interface_types_.size());
  ASSERT_TRUE(std::equal(
    controller_->state_interface_types_.begin(), controller_->state_interface_types_.end(),
                         state_interface_types_.begin(), state_interface_types_.end()));

  ASSERT_EQ(controller_->ft_sensor_name_, ft_sensor_name_);
  ASSERT_EQ(controller_->admittance_->ik_base_frame_, ik_base_frame_);
  ASSERT_EQ(controller_->admittance_->ik_tip_frame_, ik_tip_frame_);
  //   ASSERT_EQ(controller_->admittance_->ik_group_name_, ik_group_name_);
  ASSERT_EQ(controller_->admittance_->endeffector_frame_, endeffector_frame_);
  ASSERT_EQ(controller_->admittance_->fixed_world_frame_, fixed_world_frame_);
  ASSERT_EQ(controller_->admittance_->sensor_frame_, sensor_frame_);

  ASSERT_TRUE(!controller_->admittance_->selected_axes_.empty());
  ASSERT_TRUE(controller_->admittance_->selected_axes_.size() == admittance_selected_axes_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->selected_axes_.begin(), controller_->admittance_->selected_axes_.end(),
                         admittance_selected_axes_.begin(), admittance_selected_axes_.end()));

  ASSERT_TRUE(!controller_->admittance_->mass_.empty());
  ASSERT_TRUE(controller_->admittance_->mass_.size() == admittance_mass_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->mass_.begin(), controller_->admittance_->mass_.end(),
                         admittance_mass_.begin(), admittance_mass_.end()));

  ASSERT_TRUE(!controller_->admittance_->damping_.empty());
  ASSERT_TRUE(controller_->admittance_->damping_.size() == admittance_damping_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->damping_.begin(), controller_->admittance_->damping_.end(),
                         admittance_damping_.begin(), admittance_damping_.end()));

  ASSERT_TRUE(!controller_->admittance_->stiffness_.empty());
  ASSERT_TRUE(controller_->admittance_->stiffness_.size() == admittance_stiffness_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->stiffness_.begin(), controller_->admittance_->stiffness_.end(),
                         admittance_stiffness_.begin(), admittance_stiffness_.end()));
}

TEST_F(AdmittanceControllerTest, check_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), joint_command_values_.size());

  ASSERT_EQ(controller_->command_interfaces_.size(), command_interface_types_.size() * joint_names_.size());

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_interfaces.names.size(), joint_state_values_.size() + fts_state_values_.size());

  ASSERT_EQ(controller_->state_interfaces_.size(), state_interface_types_.size() * joint_names_.size() + fts_state_values_.size());
}


TEST_F(AdmittanceControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->command_interfaces_.size(), command_interface_types_.size() * joint_names_.size());

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Check ordered interfaces
  ASSERT_TRUE(!controller_->joint_command_interface_.empty());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      controller_->allowed_interface_types_.begin(), controller_->allowed_interface_types_.end(), interface);
    auto index = std::distance(controller_->allowed_interface_types_.begin(), it);

    ASSERT_TRUE(controller_->joint_command_interface_[index].size() == joint_names_.size());
  }

  ASSERT_TRUE(!controller_->joint_state_interface_.empty());
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      controller_->allowed_interface_types_.begin(), controller_->allowed_interface_types_.end(), interface);
    auto index = std::distance(controller_->allowed_interface_types_.begin(), it);

    ASSERT_TRUE(controller_->joint_state_interface_[index].size() == joint_names_.size());
  }
}

TEST_F(AdmittanceControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);
}

TEST_F(AdmittanceControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(AdmittanceControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assign_interfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);
}

TEST_F(AdmittanceControllerTest, publish_status_success)
{
  // TODO: Write also a test when Cartesian commands are used.
  SetUpController(true, true);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  // Check that force command are all zero since not used
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_force_command.wrench.force.x, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.force.y, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.force.z, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.z, 0.0);

  // Check Cartesian command message
  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.x));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.y));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.z));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.x));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.y));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.z));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.w));

  // Check joint command message
  ASSERT_TRUE(std::equal(
    msg.input_joint_command.joint_names.begin(), msg.input_joint_command.joint_names.end(),
                         joint_names_.begin(), joint_names_.end()));
  ASSERT_EQ(msg.input_joint_command.points.size(), 1u);
  ASSERT_TRUE(std::equal(msg.input_joint_command.points[0].positions.begin(), msg.input_joint_command.points[0].positions.end(), joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::find_if_not(msg.input_joint_command.points[0].velocities.begin(), msg.input_joint_command.points[0].velocities.end(),
    [](const auto & value){ return value == 0.0;}) == msg.input_joint_command.points[0].velocities.end());

  // Check messages filled from AdmittanceRule.cpp
  ASSERT_EQ(msg.measured_force.header.frame_id, sensor_frame_);
  ASSERT_EQ(msg.measured_force.wrench.force.x, fts_state_values_[0]);
  ASSERT_EQ(msg.measured_force.wrench.force.y, fts_state_values_[1]);
  ASSERT_EQ(msg.measured_force.wrench.force.z, fts_state_values_[2]);
  ASSERT_EQ(msg.measured_force.wrench.torque.x, fts_state_values_[3]);
  ASSERT_EQ(msg.measured_force.wrench.torque.y, fts_state_values_[4]);
  ASSERT_EQ(msg.measured_force.wrench.torque.z, fts_state_values_[5]);

  ASSERT_EQ(msg.measured_force_control_frame.header.frame_id, control_frame_);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.x, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.y, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.z, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.z, 0.0);

  ASSERT_EQ(msg.measured_force_endeffector_frame.header.frame_id, endeffector_frame_);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.x, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.y, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.z, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.z, 0.0);

  ASSERT_EQ(msg.desired_pose.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.x));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.y));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.z));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.x));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.y));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.z));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.w));

  ASSERT_EQ(msg.relative_desired_pose.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.x));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.y));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.z));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.x));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.y));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.z));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.w));

  // Check joint related messages
  ASSERT_TRUE(std::equal(
    msg.joint_names.begin(), msg.joint_names.end(), joint_names_.begin(), joint_names_.end()));

  ASSERT_TRUE(std::equal(
    msg.actual_joint_states.positions.begin(), msg.actual_joint_states.positions.end(),
    joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::equal(
    msg.actual_joint_states.positions.begin(), msg.actual_joint_states.positions.end(),
    joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::equal(
    msg.desired_joint_states.positions.begin(), msg.desired_joint_states.positions.end(),
                         joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::find_if_not(msg.error_joint_state.positions.begin(), msg.error_joint_state.positions.end(),
    [](const auto & value){ return value == 0.0;}) == msg.error_joint_state.positions.end());
}

TEST_F(AdmittanceControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController(true, true);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  // After first update state values should be written to command values
  ASSERT_TRUE(std::equal(joint_state_values_.begin(), joint_state_values_.end(), joint_command_values_.begin(), joint_command_values_.end()));

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_command_values_[0], 0.0, COMMON_THRESHOLD);

  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_pose_command.header.frame_id, endeffector_frame_);
}


// Add test, wrong interfaces
