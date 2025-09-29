// Copyright (c) 2025, University of Salerno, Automatic Control Group
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
// Authors: Davide Risi

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "test_gravity_compensation_pd_controller.hpp"

#include "test_asset_robot_description.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;

void GravityCompensationPDControllerTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void GravityCompensationPDControllerTest::TearDownTestCase() { rclcpp::shutdown(); }

void GravityCompensationPDControllerTest::SetUp()
{
  controller_ = std::make_unique<FriendGravityCompensationPDController>();
}

void GravityCompensationPDControllerTest::TearDown()
{
  controller_.reset(nullptr);
  command_interfaces_.clear();
  state_interfaces_.clear();
}

void GravityCompensationPDControllerTest::assign_interfaces()
{
  std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
  command_ifs.reserve(joint_command_values_.size());

  for (auto i = 0u; i < joint_command_values_.size(); ++i)
  {
    auto cmd_interface = std::make_unique<hardware_interface::CommandInterface>(
      joint_names_[i], "effort", &joint_command_values_[i]);
    command_ifs.emplace_back(*cmd_interface);
    command_interfaces_.push_back(std::move(cmd_interface));
  }

  std::vector<hardware_interface::LoanedStateInterface> state_ifs;

  // Create position state interfaces
  for (auto i = 0u; i < joint_position_values_.size(); ++i)
  {
    auto state_interface = std::make_unique<hardware_interface::StateInterface>(
      joint_names_[i], "position", &joint_position_values_[i]);
    state_ifs.emplace_back(*state_interface);
    state_interfaces_.push_back(std::move(state_interface));
  }

  // Create velocity state interfaces
  for (auto i = 0u; i < joint_velocity_values_.size(); ++i)
  {
    auto state_interface = std::make_unique<hardware_interface::StateInterface>(
      joint_names_[i], "velocity", &joint_velocity_values_[i]);
    state_ifs.emplace_back(*state_interface);
    state_interfaces_.push_back(std::move(state_interface));
  }

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

void GravityCompensationPDControllerTest::SetUpController(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  const std::string robot_description = ros2_control_test_assets::valid_robot_urdf;
  const auto result = controller_->init(
    "test_gravity_compensation_pd_controller", robot_description, 0, "", node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  assign_interfaces();
  executor_.add_node(controller_->get_node()->get_node_base_interface());
}
TEST_F(GravityCompensationPDControllerTest, ConfigureAndActivateParamsSuccess)
{
  SetUpController(
    {rclcpp::Parameter("joints", joint_names_),
     rclcpp::Parameter("p_gains", std::vector<double>{1000.0, 1000.0, 1000.0}),
     rclcpp::Parameter("d_gains", std::vector<double>{50.0, 50.0, 50.0}),
     rclcpp::Parameter("compensate_gravity", false),
     rclcpp::Parameter(
       "dynamics_solver.dynamics_solver_plugin",
       std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
     rclcpp::Parameter("dynamics_solver.tip", std::string("tool_link")),
     rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(GravityCompensationPDControllerTest, NoCommandCheckTest)
{
  SetUpController(
    {rclcpp::Parameter("joints", joint_names_),
     rclcpp::Parameter("p_gains", std::vector<double>{1000.0, 1000.0, 1000.0}),
     rclcpp::Parameter("d_gains", std::vector<double>{50.0, 50.0, 50.0}),
     rclcpp::Parameter("compensate_gravity", false),
     rclcpp::Parameter(
       "dynamics_solver.dynamics_solver_plugin",
       std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
     rclcpp::Parameter("dynamics_solver.tip", std::string("tool_link")),
     rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))});
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  controller_->on_export_reference_interfaces();

  // update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check joint commands are still the default ones
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-6);
}

TEST_F(GravityCompensationPDControllerTest, StopJointsOnDeactivateTest)
{
  SetUpController(
    {rclcpp::Parameter("joints", joint_names_),
     rclcpp::Parameter("p_gains", std::vector<double>{1000.0, 1000.0, 1000.0}),
     rclcpp::Parameter("d_gains", std::vector<double>{50.0, 50.0, 50.0}),
     rclcpp::Parameter("compensate_gravity", false),
     rclcpp::Parameter(
       "dynamics_solver.dynamics_solver_plugin",
       std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
     rclcpp::Parameter("dynamics_solver.tip", std::string("tool_link")),
     rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are still the default ones
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-6);

  // stop the controller
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are now zero
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-6);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-6);
}
