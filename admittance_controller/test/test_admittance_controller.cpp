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


// Test on_configure returns ERROR when a required parameter is missing
TEST_P(AdmittanceControllerTestParameterizedMissingParameters, one_parameter_is_missing)
{
  ASSERT_EQ(SetUpController(GetParam()), controller_interface::return_type::ERROR);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

INSTANTIATE_TEST_SUITE_P(
    MissingMandatoryParameterDuringConfiguration,
    AdmittanceControllerTestParameterizedMissingParameters,
    ::testing::Values(
        "admittance.mass",
        "admittance.selected_axes",
        "admittance.stiffness",
        "chainable_command_interfaces",
        "command_interfaces",
        "control.frame.id",
        "fixed_world_frame.frame.id",
        "ft_sensor.frame.id",
        "ft_sensor.name",
        "gravity_compensation.CoG.pos",
        "gravity_compensation.frame.id",
        "joints",
        "kinematics.base",
        "kinematics.plugin_name",
        "kinematics.plugin_package",
        "kinematics.tip",
        "state_interfaces"
    )
);

INSTANTIATE_TEST_SUITE_P(
    InvalidParameterDuringConfiguration,
    AdmittanceControllerTestParameterizedInvalidParameters,
    ::testing::Values(
    // wrong length COG
    std::make_tuple(
          std::string("gravity_compensation.CoG.pos"),
          rclcpp::ParameterValue(std::vector<double>()={1,2,3,4})
    ),
    // wrong length stiffness
    std::make_tuple(
        std::string("admittance.stiffness"),
        rclcpp::ParameterValue(std::vector<double>()={1,2,3})
    ),
      // negative stiffness
    std::make_tuple(
        std::string("admittance.stiffness"),
        rclcpp::ParameterValue(std::vector<double>()={-1,-2,3,4,5,6})
    ),
    // wrong length mass
    std::make_tuple(
        std::string("admittance.mass"),
        rclcpp::ParameterValue(std::vector<double>()={1,2,3})
    ),
    // negative mass
    std::make_tuple(
        std::string("admittance.mass"),
        rclcpp::ParameterValue(std::vector<double>()={-1,-2,3,4,5,6})
    ),
    // wrong length damping ratio
    std::make_tuple(
        std::string("admittance.damping_ratio"),
        rclcpp::ParameterValue(std::vector<double>()={1,2,3})
    ),
    // wrong length selected axes
    std::make_tuple(
        std::string("admittance.selected_axes"),
        rclcpp::ParameterValue(std::vector<double>()={1,2,3})
    ),
    // invalid robot description
    std::make_tuple(
        std::string("robot_description"),
        rclcpp::ParameterValue(std::string()="bad_robot")
    )
  )
);

TEST_F(AdmittanceControllerTest, all_parameters_set_configure_success)
{

  auto result = SetUpController();

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(!controller_->admittance_->parameters_.joints.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.joints.size() == joint_names_.size());
  ASSERT_TRUE(std::equal(controller_->admittance_->parameters_.joints.begin(), controller_->admittance_->parameters_.joints.end(),
    joint_names_.begin(), joint_names_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.command_interfaces.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.command_interfaces.size() == command_interface_types_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->parameters_.command_interfaces.begin(), controller_->admittance_->parameters_.command_interfaces.end(),
    command_interface_types_.begin(), command_interface_types_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.state_interfaces.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.state_interfaces.size() == state_interface_types_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->parameters_.state_interfaces.begin(), controller_->admittance_->parameters_.state_interfaces.end(),
                         state_interface_types_.begin(), state_interface_types_.end()));

  ASSERT_EQ(controller_->admittance_->parameters_.ft_sensor.name, ft_sensor_name_);
  ASSERT_EQ(controller_->admittance_->parameters_.kinematics.base, ik_base_frame_);
  ASSERT_EQ(controller_->admittance_->parameters_.ft_sensor.frame.id, sensor_frame_);

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.selected_axes.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.admittance.selected_axes.size() == admittance_selected_axes_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->parameters_.admittance.selected_axes.begin(),
    controller_->admittance_->parameters_.admittance.selected_axes.end(),
                         admittance_selected_axes_.begin(), admittance_selected_axes_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.mass.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.admittance.mass.size() == admittance_mass_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->parameters_.admittance.mass.begin(), controller_->admittance_->parameters_.admittance.mass.end(),
                         admittance_mass_.begin(), admittance_mass_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.damping_ratio.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.admittance.damping_ratio.size() == admittance_damping_ratio_.size());
  ASSERT_TRUE(std::equal(
      controller_->admittance_->parameters_.admittance.damping_ratio.begin(),
      controller_->admittance_->parameters_.admittance.damping_ratio.end(),
      admittance_damping_ratio_.begin(), admittance_damping_ratio_.end()));

  ASSERT_TRUE(!controller_->admittance_->parameters_.admittance.stiffness.empty());
  ASSERT_TRUE(controller_->admittance_->parameters_.admittance.stiffness.size() == admittance_stiffness_.size());
  ASSERT_TRUE(std::equal(
    controller_->admittance_->parameters_.admittance.stiffness.begin(),
    controller_->admittance_->parameters_.admittance.stiffness.end(),
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

}

TEST_F(AdmittanceControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
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
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
}

// TODO (pac48) this test needs to be enabled and edited once a admittance state message definition is finalized
//TEST_F(AdmittanceControllerTest, publish_status_success)
//{
//
//  SetUpController();
//
//  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
//  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
//
//  broadcast_tfs();
//  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::OK);
//
//  ControllerStateMsg msg;
//  subscribe_and_get_messages(msg);
//
//  // Check that wrench command are all zero since not used
//  ASSERT_EQ(msg.input_wrench_command.header.frame_id, control_frame_);
//  ASSERT_EQ(msg.input_wrench_command.wrench.force.x, 0.0);
//  ASSERT_EQ(msg.input_wrench_command.wrench.force.y, 0.0);
//  ASSERT_EQ(msg.input_wrench_command.wrench.force.z, 0.0);
//  ASSERT_EQ(msg.input_wrench_command.wrench.torque.x, 0.0);
//  ASSERT_EQ(msg.input_wrench_command.wrench.torque.y, 0.0);
//  ASSERT_EQ(msg.input_wrench_command.wrench.torque.z, 0.0);
//
//  // Check Cartesian command message
//  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.x));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.y));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.z));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.x));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.y));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.z));
//  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.w));
//
//  // Check joint command message
//  ASSERT_TRUE(std::equal(
//    msg.input_joint_command.joint_names.begin(), msg.input_joint_command.joint_names.end(),
//                         joint_names_.begin(), joint_names_.end()));
//  ASSERT_EQ(msg.input_joint_command.points.size(), 1u);
//  ASSERT_TRUE(std::equal(msg.input_joint_command.points[0].positions.begin(), msg.input_joint_command.points[0].positions.end(), joint_state_values_.begin(), joint_state_values_.end()));
//
//  ASSERT_TRUE(std::find_if_not(msg.input_joint_command.points[0].velocities.begin(), msg.input_joint_command.points[0].velocities.end(),
//    [](const auto & value){ return value == 0.0;}) == msg.input_joint_command.points[0].velocities.end());
//
//  // Check messages filled from AdmittanceRule.cpp
//  ASSERT_EQ(msg.measured_wrench.header.frame_id, sensor_frame_);
//  ASSERT_EQ(msg.measured_wrench.wrench.force.x, fts_state_values_[0]);
//  ASSERT_EQ(msg.measured_wrench.wrench.force.y, fts_state_values_[1]);
//  ASSERT_EQ(msg.measured_wrench.wrench.force.z, fts_state_values_[2]);
//  ASSERT_EQ(msg.measured_wrench.wrench.torque.x, fts_state_values_[3]);
//  ASSERT_EQ(msg.measured_wrench.wrench.torque.y, fts_state_values_[4]);
//  ASSERT_EQ(msg.measured_wrench.wrench.torque.z, fts_state_values_[5]);
//
//  ASSERT_EQ(msg.measured_wrench_control_frame.header.frame_id, control_frame_);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.force.x, 0.0);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.force.y, 0.0);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.force.z, 0.0);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.torque.x, 0.0);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.torque.y, 0.0);
//  ASSERT_EQ(msg.measured_wrench_control_frame.wrench.torque.z, 0.0);
//
//  ASSERT_EQ(msg.desired_pose.header.frame_id, control_frame_);
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.x));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.y));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.z));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.x));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.y));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.z));
//  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.w));
//
//  ASSERT_EQ(msg.relative_desired_pose.header.frame_id, control_frame_);
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.x));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.y));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.z));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.x));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.y));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.z));
//  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.w));
//
//  // Check joint related messages
//  ASSERT_TRUE(std::equal(
//    msg.joint_names.begin(), msg.joint_names.end(), joint_names_.begin(), joint_names_.end()));
//
//  ASSERT_TRUE(std::equal(
//    msg.actual_joint_state.positions.begin(), msg.actual_joint_state.positions.end(),
//    joint_state_values_.begin(), joint_state_values_.end()));
//
//  ASSERT_TRUE(std::equal(
//    msg.actual_joint_state.positions.begin(), msg.actual_joint_state.positions.end(),
//    joint_state_values_.begin(), joint_state_values_.end()));
//
//  ASSERT_TRUE(std::equal(
//    msg.desired_joint_state.positions.begin(), msg.desired_joint_state.positions.end(),
//                         joint_state_values_.begin(), joint_state_values_.end()));
//
//  ASSERT_TRUE(std::find_if_not(msg.error_joint_state.positions.begin(), msg.error_joint_state.positions.end(),
//    [](const auto & value){ return value == 0.0;}) == msg.error_joint_state.positions.end());
//}

TEST_F(AdmittanceControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::OK);

  // After first update state, commanded position should be near the start state
  for (auto i = 0ul; i < joint_state_values_.size(); i++){
    ASSERT_NEAR(joint_state_values_[i], joint_command_values_[i], COMMON_THRESHOLD);
  }

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_wrench_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  broadcast_tfs();
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_command_values_[0], joint_state_values_[0], COMMON_THRESHOLD);

  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_wrench_command.header.frame_id, control_frame_);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result =  RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// Add test, wrong interfaces
