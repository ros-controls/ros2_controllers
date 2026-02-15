// Copyright (C) 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "admittance_controller/admittance_rule.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

namespace admittance_controller
{
namespace test
{

// Type aliases for Eigen types to avoid comma issues in MOCK_METHOD
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Jacobian6xN = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using JacobianNx6 = Eigen::Matrix<double, Eigen::Dynamic, 6>;

// Mock kinematics interface for testing admittance rule without real robot kinematics
// Uses identity transformations to isolate mass matrix transformation logic
class MockKinematicsInterface : public kinematics_interface::KinematicsInterface
{
public:
  MOCK_METHOD(
    bool, initialize,
    (const std::string &, std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>,
     const std::string &),
    (override));
  MOCK_METHOD(
    bool, calculate_link_transform,
    (const Eigen::VectorXd &, const std::string &, Eigen::Isometry3d &), (override));
  MOCK_METHOD(
    bool, convert_cartesian_deltas_to_joint_deltas,
    (const Eigen::VectorXd &, const Vector6d &, const std::string &, Eigen::VectorXd &),
    (override));
  MOCK_METHOD(
    bool, convert_joint_deltas_to_cartesian_deltas,
    (const Eigen::VectorXd &, const Eigen::VectorXd &, const std::string &, Vector6d &),
    (override));
  MOCK_METHOD(
    bool, calculate_jacobian, (const Eigen::VectorXd &, const std::string &, Jacobian6xN &),
    (override));
  MOCK_METHOD(
    bool, calculate_jacobian_inverse, (const Eigen::VectorXd &, const std::string &, JacobianNx6 &),
    (override));
  MOCK_METHOD(
    bool, calculate_frame_difference, (Vector7d &, Vector7d &, double, Vector6d &), (override));
};

// Testable AdmittanceRule exposes protected members for direct testing
class TestableAdmittanceRule : public AdmittanceRule
{
public:
  explicit TestableAdmittanceRule(
    const std::shared_ptr<admittance_controller::ParamListener> & parameter_handler)
  : AdmittanceRule(parameter_handler)
  {
  }

  // Expose protected members for testing
  using AdmittanceRule::calculate_admittance_rule;
  using AdmittanceRule::kinematics_;
  using AdmittanceRule::parameters_;
};

class MassMatrixTransformationTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    // Initialize node with required parameters for admittance controller
    createNodeWithParameters();

    // Create admittance rule with mock kinematics
    param_listener_ = std::make_shared<admittance_controller::ParamListener>(node_);
    admittance_rule_ = std::make_unique<TestableAdmittanceRule>(param_listener_);

    // Inject mock kinematics interface
    auto mock_kinematics = std::make_unique<MockKinematicsInterface>();
    mock_kinematics_ptr_ = mock_kinematics.get();
    admittance_rule_->kinematics_ = std::move(mock_kinematics);
  }

private:
  // Creates ROS node and declares all required parameters
  void createNodeWithParameters()
  {
    std::vector<rclcpp::Parameter> params = {
      {"joints",
       std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}},
      {"command_interfaces", std::vector<std::string>{"position"}},
      {"state_interfaces", std::vector<std::string>{"position", "velocity"}},
      {"chainable_command_interfaces", std::vector<std::string>{"position", "velocity"}},
      {"admittance.mass", std::vector<double>{1.0, 1.0, 1.0, 0.05, 0.05, 0.05}},
      {"admittance.stiffness", std::vector<double>{50.0, 50.0, 50.0, 1.0, 1.0, 1.0}},
      {"admittance.damping_ratio", std::vector<double>{0.7, 0.7, 0.7, 0.7, 0.7, 0.7}},
      {"admittance.selected_axes", std::vector<bool>{true, true, true, true, true, true}},
      {"admittance.joint_damping", 5.0},
      {"kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")},
      {"kinematics.plugin_package", std::string("kinematics_interface")},
      {"kinematics.base", std::string("base_link")},
      {"kinematics.tip", std::string("tool0")},
      {"ft_sensor.name", std::string("ft_sensor")},
      {"ft_sensor.frame.id", std::string("link_6")},
      {"ft_sensor.filter_coefficient", 0.05},
      {"control.frame.id", std::string("tool0")},
      {"fixed_world_frame.frame.id", std::string("base_link")},
      {"gravity_compensation.frame.id", std::string("tool0")},
      {"gravity_compensation.CoG.pos", std::vector<double>{0.0, 0.0, 0.0}},
      {"gravity_compensation.CoG.force", 0.0}};

    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("test_node", options);

    // Ensure all parameters are properly declared
    for (const auto & param : params)
    {
      if (!node_->has_parameter(param.get_name()))
      {
        node_->declare_parameter(param.get_name(), param.get_parameter_value());
      }
    }
  }

protected:
  // Configure mock kinematics to use identity transformations
  // This isolates mass matrix transformation logic from actual robot kinematics
  void setupMockKinematics()
  {
    // All transforms are identity - no rotation or translation
    EXPECT_CALL(
      *mock_kinematics_ptr_, calculate_link_transform(::testing::_, ::testing::_, ::testing::_))
      .WillRepeatedly(
        ::testing::DoAll(
          ::testing::SetArgReferee<2>(Eigen::Isometry3d::Identity()), ::testing::Return(true)));

    // Use identity Jacobian: cartesian deltas = joint deltas
    // This allows mass matrix effects to propagate directly to joint space
    EXPECT_CALL(
      *mock_kinematics_ptr_, convert_cartesian_deltas_to_joint_deltas(
                               ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillRepeatedly(
        ::testing::Invoke(
          [](
            const Eigen::VectorXd &, const Vector6d & cartesian_deltas, const std::string &,
            Eigen::VectorXd & joint_deltas) -> bool
          {
            joint_deltas = cartesian_deltas;
            return true;
          }));

    // Mock inverse kinematics: convert joint space back to Cartesian space
    // Used by admittance rule to update admittance_velocity from joint_vel
    EXPECT_CALL(
      *mock_kinematics_ptr_, convert_joint_deltas_to_cartesian_deltas(
                               ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillRepeatedly(
        ::testing::Invoke(
          [](
            const Eigen::VectorXd &, const Eigen::VectorXd & joint_deltas, const std::string &,
            Vector6d & cartesian_deltas) -> bool
          {
            cartesian_deltas = joint_deltas;
            return true;
          }));
  }

  // Create admittance state with specified parameters
  AdmittanceState createAdmittanceState(
    const Eigen::Matrix3d & rot_base_control, const Vector6d & mass, const Vector6d & stiffness,
    const Vector6d & damping, const Vector6d & wrench_base)
  {
    AdmittanceState state(6);
    state.rot_base_control = rot_base_control;
    state.mass = mass;

    // Calculate inverse mass for admittance dynamics
    for (Eigen::Index i = 0; i < 6; ++i)
    {
      state.mass_inv[i] = 1.0 / state.mass[i];
    }

    state.stiffness = stiffness;
    state.damping = damping;
    state.selected_axes = Vector6d::Ones();  // All axes active
    state.current_joint_pos = Eigen::VectorXd::Zero(6);
    state.joint_pos = Eigen::VectorXd::Zero(6);
    state.joint_vel = Eigen::VectorXd::Zero(6);
    state.joint_acc = Eigen::VectorXd::Zero(6);
    state.admittance_velocity = Vector6d::Zero();
    state.admittance_acceleration = Vector6d::Zero();
    state.wrench_base = wrench_base;
    state.ref_trans_base_ft = Eigen::Isometry3d::Identity();
    state.ft_sensor_frame = "link_6";

    return state;
  }

  // Get rotation matrix that maps control z-axis to base x-axis
  // This tests that mass in control frame correctly affects motion in base frame
  Eigen::Matrix3d getControlFrameRotation()
  {
    // 90 degree rotation around Y axis: control z -> base x
    return Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  }

  // Helper to create standard parameters for tests
  Vector6d createTranslationalParameters(double x, double y, double z)
  {
    Vector6d params;
    params << x, y, z, DEFAULT_ROT_VALUE, DEFAULT_ROT_VALUE, DEFAULT_ROT_VALUE;
    return params;
  }

  // Test parameters - organized by domain
  static constexpr double DEFAULT_DT = 0.01;  // Time step for integration

  // Translational dynamics
  static constexpr double DEFAULT_MASS = 1.0;
  static constexpr double HIGH_MASS = 100.0;
  static constexpr double DEFAULT_STIFFNESS = 50.0;
  static constexpr double DEFAULT_DAMPING = 10.0;
  static constexpr double DEFAULT_FORCE = 5.0;

  // Rotational dynamics (typically smaller values)
  static constexpr double DEFAULT_ROT_VALUE = 0.1;  // Generic rotational parameter
  static constexpr double DEFAULT_ROT_MASS = 0.05;
  static constexpr double DEFAULT_ROT_STIFFNESS = 1.0;
  static constexpr double DEFAULT_ROT_DAMPING = 0.1;

  // Test fixture members
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<admittance_controller::ParamListener> param_listener_;
  std::unique_ptr<TestableAdmittanceRule> admittance_rule_;
  MockKinematicsInterface * mock_kinematics_ptr_;
};

// Test 1: Verify that calculate_admittance_rule succeeds with rotated control frame
// and produces motion when force is applied
// cppcheck-suppress syntaxError
// To suppress the warning reported by cppcheck for complex macro expansion
TEST_F(MassMatrixTransformationTest, mass_matrix_transformation_consistency)
{
  // Setup: Control frame rotated so control z-axis aligns with base x-axis
  Eigen::Matrix3d rot_base_control = getControlFrameRotation();

  // High mass in control z-direction (which maps to base x-direction)
  Vector6d mass = createTranslationalParameters(DEFAULT_MASS, DEFAULT_MASS, HIGH_MASS);
  Vector6d stiffness =
    createTranslationalParameters(DEFAULT_STIFFNESS, DEFAULT_STIFFNESS, DEFAULT_STIFFNESS);
  Vector6d damping =
    createTranslationalParameters(DEFAULT_DAMPING, DEFAULT_DAMPING, DEFAULT_DAMPING);

  // Force applied in base x-direction
  Vector6d wrench;
  wrench << DEFAULT_FORCE, 0.0, 0.0, 0.0, 0.0, 0.0;

  AdmittanceState state = createAdmittanceState(rot_base_control, mass, stiffness, damping, wrench);
  setupMockKinematics();

  // Execute
  bool success = admittance_rule_->calculate_admittance_rule(state, DEFAULT_DT);

  // Verify
  EXPECT_TRUE(success) << "calculate_admittance_rule should succeed with rotated control frame";
  EXPECT_GT(state.joint_pos.norm(), 0.0) << "Should produce motion when force is applied";
}

// Test 2: Verify that higher mass in control frame results in less motion
// Physics expectation: F = ma, so higher mass -> lower acceleration -> less motion
TEST_F(MassMatrixTransformationTest, mass_affects_motion_in_rotated_frame)
{
  // Setup: Control frame rotated so control z-axis aligns with base x-axis
  Eigen::Matrix3d rot_base_control = getControlFrameRotation();

  Vector6d stiffness =
    createTranslationalParameters(DEFAULT_STIFFNESS, DEFAULT_STIFFNESS, DEFAULT_STIFFNESS);
  Vector6d damping =
    createTranslationalParameters(DEFAULT_DAMPING, DEFAULT_DAMPING, DEFAULT_DAMPING);

  // Force applied in base x-direction
  Vector6d wrench;
  wrench << DEFAULT_FORCE, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Case 1: Low mass in control z-direction
  Vector6d mass_low = createTranslationalParameters(DEFAULT_MASS, DEFAULT_MASS, DEFAULT_MASS);
  AdmittanceState state_low =
    createAdmittanceState(rot_base_control, mass_low, stiffness, damping, wrench);

  // Case 2: High mass in control z-direction
  Vector6d mass_high = createTranslationalParameters(DEFAULT_MASS, DEFAULT_MASS, HIGH_MASS);
  AdmittanceState state_high =
    createAdmittanceState(rot_base_control, mass_high, stiffness, damping, wrench);

  setupMockKinematics();

  // Execute
  bool success_low = admittance_rule_->calculate_admittance_rule(state_low, DEFAULT_DT);
  bool success_high = admittance_rule_->calculate_admittance_rule(state_high, DEFAULT_DT);

  // Verify both succeed
  EXPECT_TRUE(success_low) << "Low mass case should succeed";
  EXPECT_TRUE(success_high) << "High mass case should succeed";

  // Verify physics: higher mass -> less motion
  double motion_low = state_low.joint_pos.norm();
  double motion_high = state_high.joint_pos.norm();

  EXPECT_GT(motion_low, motion_high) << "Higher mass should result in smaller motion (F=ma)";
}

// Test 3: Verify mass matrix transformation from control frame to base frame
// Test scenario: With control frame rotated (control z -> base x), varying the mass
// in control z should affect the response to force applied in base x direction
TEST_F(MassMatrixTransformationTest, mass_transformation_affects_base_frame_response)
{
  // Setup: Control frame rotated 90° around Y-axis
  // This means control z-axis is aligned with base x-axis
  Eigen::Matrix3d rot_base_control = getControlFrameRotation();

  Vector6d stiffness =
    createTranslationalParameters(DEFAULT_STIFFNESS, DEFAULT_STIFFNESS, DEFAULT_STIFFNESS);
  Vector6d damping =
    createTranslationalParameters(DEFAULT_DAMPING, DEFAULT_DAMPING, DEFAULT_DAMPING);

  // Force applied in base x-direction
  Vector6d wrench;
  wrench << DEFAULT_FORCE, 0.0, 0.0, 0.0, 0.0, 0.0;

  setupMockKinematics();

  // Case 1: Low mass in control z (which is aligned with base x)
  Vector6d mass_low = createTranslationalParameters(DEFAULT_MASS, DEFAULT_MASS, DEFAULT_MASS);
  AdmittanceState state_low =
    createAdmittanceState(rot_base_control, mass_low, stiffness, damping, wrench);
  bool success_low = admittance_rule_->calculate_admittance_rule(state_low, DEFAULT_DT);
  EXPECT_TRUE(success_low) << "Low mass case should succeed";

  // Case 2: High mass in control z (which is aligned with base x)
  Vector6d mass_high = createTranslationalParameters(DEFAULT_MASS, DEFAULT_MASS, HIGH_MASS);
  AdmittanceState state_high =
    createAdmittanceState(rot_base_control, mass_high, stiffness, damping, wrench);
  bool success_high = admittance_rule_->calculate_admittance_rule(state_high, DEFAULT_DT);
  EXPECT_TRUE(success_high) << "High mass case should succeed";

  // Verify the transformation is working correctly
  double motion_low = state_low.joint_pos.norm();
  double motion_high = state_high.joint_pos.norm();

  // Key assertion: Mass in control z affects motion when force is in base x
  // This proves the mass matrix is correctly transformed from control to base frame
  EXPECT_GT(motion_low, motion_high)
    << "Mass in control z should affect response to force in base x, "
    << "demonstrating correct mass matrix transformation";

  // Sanity checks: both cases should produce non-zero motion
  EXPECT_GT(motion_low, 0.0) << "Low mass should produce measurable motion";
  EXPECT_GT(motion_high, 0.0) << "High mass should still produce some motion";
}

}  // namespace test
}  // namespace admittance_controller

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
