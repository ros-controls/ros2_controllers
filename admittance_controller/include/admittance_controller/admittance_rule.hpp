// Copyright (c) 2021, PickNik, Inc.
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
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_controller_parameters.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace admittance_controller
{
struct AdmittanceState
{
  explicit AdmittanceState(size_t num_joints)
  {
    admittance_velocity.setZero();
    admittance_acceleration.setZero();
    damping.setZero();
    mass.setOnes();
    mass_inv.setZero();
    stiffness.setZero();
    selected_axes.setZero();
    auto idx = static_cast<Eigen::Index>(num_joints);
    current_joint_pos = Eigen::VectorXd::Zero(idx);
    joint_pos = Eigen::VectorXd::Zero(idx);
    joint_vel = Eigen::VectorXd::Zero(idx);
    joint_acc = Eigen::VectorXd::Zero(idx);
  }

  Eigen::VectorXd current_joint_pos;
  Eigen::VectorXd joint_pos;
  Eigen::VectorXd joint_vel;
  Eigen::VectorXd joint_acc;
  Eigen::Matrix<double, 6, 1> damping;
  Eigen::Matrix<double, 6, 1> mass;
  Eigen::Matrix<double, 6, 1> mass_inv;
  Eigen::Matrix<double, 6, 1> selected_axes;
  Eigen::Matrix<double, 6, 1> stiffness;
  Eigen::Matrix<double, 6, 1> wrench_base;
  Eigen::Matrix<double, 6, 1> admittance_acceleration;
  Eigen::Matrix<double, 6, 1> admittance_velocity;
  Eigen::Isometry3d admittance_position;
  Eigen::Matrix<double, 3, 3> rot_base_control;
  Eigen::Isometry3d ref_trans_base_ft;
  std::string ft_sensor_frame;
};

class AdmittanceRule
{
public:
  explicit AdmittanceRule(
    const std::shared_ptr<admittance_controller::ParamListener> & parameter_handler)
  {
    parameter_handler_ = parameter_handler;
    parameters_ = parameter_handler_->get_params();
    num_joints_ = parameters_.joints.size();
    admittance_state_ = AdmittanceState(num_joints_);
    reset(num_joints_);
  }

  /// Configure admittance rule memory using number of joints.
  controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const size_t num_joint,
    const std::string & robot_description);

  /// Reset all values back to default
  controller_interface::return_type reset(const size_t num_joints);

  /**
   * Updates parameter_ struct if any parameters have changed since last update. Parameter dependent
   * Eigen field members (end_effector_weight_, cog_pos_, mass_, mass_inv_ stiffness, selected_axes,
   * damping_) are also updated
   */
  void apply_parameters_update();

  /**
   * Calculate 'desired joint states' based on the 'measured force', 'reference joint state', and
   * 'current_joint_state'.
   *
   * All transforms (e.g., world to base, sensor to base, CoG to base) are now computed
   * directly in this function and stored in `admittance_state_`, removing the
   * need for an intermediate transform struct.
   *
   * All transforms (e.g., world to base, sensor to base, CoG to base) are now computed
   * directly in this function and stored in `admittance_state_`, removing the
   * need for an intermediate transform struct.
   *
   * \param[in] current_joint_state current joint state of the robot
   * \param[in] measured_wrench most recent measured wrench from force torque sensor
   * \param[in] reference_joint_state input joint state reference
   * \param[in] period time in seconds since last controller update
   * \param[out] desired_joint_state joint state reference after the admittance offset is applied to
   * the input reference
   */
  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states);

  /**
   * Set fields of `state_message` from current admittance controller state.
   *
   * \param[out] state_message message containing target position/vel/accel, wrench, and actual
   * robot state, among other things
   */
  const control_msgs::msg::AdmittanceControllerState & get_controller_state();

public:
  // admittance config parameters
  std::shared_ptr<admittance_controller::ParamListener> parameter_handler_;
  admittance_controller::Params parameters_;

protected:
  /**
   * Calculates the admittance rule from given the robot's current joint angles. The admittance
   * controller state input is updated with the new calculated values. A boolean value is returned
   * indicating if any of the kinematics plugin calls failed. \param[in] admittance_state contains
   * all the information needed to calculate the admittance offset \param[in] dt controller period
   * \param[out] success true if no calls to the kinematics interface fail
   */
  bool calculate_admittance_rule(AdmittanceState & admittance_state, double dt);

  /**
   * Updates internal estimate of wrench in world frame `wrench_world_` given the new measurement
   * `measured_wrench`, the sensor to base frame rotation `sensor_world_rot`, and the center of
   * gravity frame to base frame rotation `cog_world_rot`. The `wrench_world_` estimate includes
   * gravity compensation \param[in] measured_wrench  most recent measured wrench from force torque
   * sensor \param[in] sensor_world_rot rotation matrix from world frame to sensor frame \param[in]
   * cog_world_rot rotation matrix from world frame to center of gravity frame
   */
  void process_wrench_measurements(
    const geometry_msgs::msg::Wrench & measured_wrench,
    const Eigen::Matrix<double, 3, 3> & sensor_world_rot,
    const Eigen::Matrix<double, 3, 3> & cog_world_rot);

  template <typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

  // number of robot joint
  size_t num_joints_;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
    kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // filtered wrench in world frame
  Eigen::Matrix<double, 6, 1> wrench_world_;

  // admittance controllers internal state
  AdmittanceState admittance_state_{0};

  // position of center of gravity in cog_frame
  Eigen::Vector3d cog_pos_;

  // force applied to sensor due to weight of end effector
  Eigen::Vector3d end_effector_weight_;

  // ROS
  control_msgs::msg::AdmittanceControllerState state_message_;
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
