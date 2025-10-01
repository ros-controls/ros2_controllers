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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "test_gravity_compensation_pd_controller.hpp"

#include "test_asset_robot_description.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;

// ****** Definitions of the test cases for GravityCompensationPDController public API ******

void GravityCompensationPDControllerTest::SetUp()
{
  controller_ = std::make_unique<FriendGravityCompensationPDController>();
  std::vector<rclcpp::Parameter> default_parameters = {
    rclcpp::Parameter("joints", joint_names_),
    rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
    rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
    rclcpp::Parameter("compensate_gravity", true),
    rclcpp::Parameter(
      "dynamics_solver.dynamics_solver_plugin",
      std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
    rclcpp::Parameter("dynamics_solver.tip", std::string("link3")),
    rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))};
  SetUpController(default_parameters);
}

void GravityCompensationPDControllerTest::TearDown()
{
  controller_.reset(nullptr);
  command_interfaces_.clear();
  state_interfaces_.clear();
}

void GravityCompensationPDControllerTestBase::assign_interfaces_()
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

void GravityCompensationPDControllerTestBase::SetUpController(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  const auto result = controller_->init(
    "test_gravity_compensation_pd_controller", ros2_control_test_assets::valid_robot_urdf, 0, "",
    node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  assign_interfaces_();
  executor_.add_node(controller_->get_node()->get_node_base_interface());
}

TEST_F(GravityCompensationPDControllerTest, ConfigureAndActivateParamsSuccess)
{
  ASSERT_EQ(controller_->on_init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(GravityCompensationPDControllerTest, NoCommandCheckTest)
{
  ASSERT_EQ(controller_->on_init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // This method is protected, so we need the friend class
  controller_->on_export_reference_interfaces();

  // Update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // Check joint commands with gravity compensation only and no PD action
  EXPECT_NEAR(joint_command_values_[0], 0.1, 1e-9);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-9);
}

TEST_F(GravityCompensationPDControllerTest, StopJointsOnDeactivateTest)
{
  ASSERT_EQ(controller_->on_init(), CallbackReturn::SUCCESS);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are still the default ones
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-9);

  // stop the controller
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are now zero
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-9);
}

TEST_F(GravityCompensationPDControllerTest, CommandNanOnErrorTest)
{
  ASSERT_EQ(controller_->on_init(), CallbackReturn::SUCCESS);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are still the default ones
  EXPECT_NEAR(joint_command_values_[0], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[1], 0.0, 1e-9);
  EXPECT_NEAR(joint_command_values_[2], 0.0, 1e-9);

  // stop the controller
  ASSERT_EQ(controller_->on_error(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are now NaN
  EXPECT_TRUE(std::isnan(joint_command_values_[0]));
  EXPECT_TRUE(std::isnan(joint_command_values_[1]));
  EXPECT_TRUE(std::isnan(joint_command_values_[2]));
}

// ****** Definitions of the test cases for invalid parameters ******

void GravityCompensationPDControllerInvalidParameterTest::SetUp()
{
  controller_ = std::make_unique<FriendGravityCompensationPDController>();
  // Parameters are set in each test case
  const std::vector<rclcpp::Parameter> parameters = WithParamInterface::GetParam();
  SetUpController(parameters);
}

void GravityCompensationPDControllerInvalidParameterTest::TearDown()
{
  controller_.reset(nullptr);
  command_interfaces_.clear();
  state_interfaces_.clear();
}

TEST_P(GravityCompensationPDControllerInvalidParameterTest, ConfigureFail)
{
  // configure should fail
  ASSERT_EQ(controller_->on_init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidConfigParameters, GravityCompensationPDControllerInvalidParameterTest,
  testing::Values(
    // Empty joint names
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter(
        "dynamics_solver.dynamics_solver_plugin",
        std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
      rclcpp::Parameter("dynamics_solver.tip", std::string("link3")),
      rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))},

    // Mismatched p_gains size
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1", "joint2", "joint3"}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter(
        "dynamics_solver.dynamics_solver_plugin",
        std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
      rclcpp::Parameter("dynamics_solver.tip", std::string("link3")),
      rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))},

    // Mismatched d_gains size
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1", "joint2", "joint3"}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter(
        "dynamics_solver.dynamics_solver_plugin",
        std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
      rclcpp::Parameter("dynamics_solver.tip", std::string("link3")),
      rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))}));

// ****** Definitions of the test cases for missing parameters ******

void GravityCompensationPDControllerMissingParameterTest::SetUp()
{
  controller_ = std::make_unique<FriendGravityCompensationPDController>();
  // Parameters are set in each test case
  const std::vector<rclcpp::Parameter> parameters = WithParamInterface::GetParam();
  auto node_options = controller_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  controller_->init(
    "test_gravity_compensation_pd_controller", ros2_control_test_assets::valid_robot_urdf, 0, "",
    node_options);

  executor_.add_node(controller_->get_node()->get_node_base_interface());
}

TEST_P(GravityCompensationPDControllerMissingParameterTest, InitFail)
{
  // init should fail
  ASSERT_EQ(controller_->on_init(), CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidInitParameters, GravityCompensationPDControllerMissingParameterTest,
  testing::Values(
    // Missing dynamics solver plugin
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1", "joint2", "joint3"}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter("dynamics_solver.tip", std::string("link3")),
      rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))},
    // Missing dynamics solver tip
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1", "joint2", "joint3"}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter(
        "dynamics_solver.dynamics_solver_plugin",
        std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
      rclcpp::Parameter("dynamics_solver.root", std::string("base_link"))},
    // Missing dynamics solver root
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1", "joint2", "joint3"}),
      rclcpp::Parameter("p_gains", std::vector<double>{100.0, 100.0, 100.0}),
      rclcpp::Parameter("d_gains", std::vector<double>{5.0, 5.0, 5.0}),
      rclcpp::Parameter("compensate_gravity", true),
      rclcpp::Parameter(
        "dynamics_solver.dynamics_solver_plugin",
        std::string("kdl_inverse_dynamics_solver/InverseDynamicsSolverKDL")),
      rclcpp::Parameter("dynamics_solver.tip", std::string("link3"))}));

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
