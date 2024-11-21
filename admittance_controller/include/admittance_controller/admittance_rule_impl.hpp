// Copyright (c) 2022, PickNik, Inc.
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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_

#include "admittance_controller/admittance_rule.hpp"

#include <memory>
#include <string>
#include <vector>

#include <control_toolbox/filters.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "rclcpp/duration.hpp"

namespace admittance_controller
{

constexpr auto NUM_CARTESIAN_DOF = 6;  // (3 translation + 3 rotation)

/// Configure admittance rule memory for num joints and load kinematics interface
controller_interface::return_type AdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const size_t num_joints)
{
  num_joints_ = num_joints;

  // initialize memory and values to zero  (non-realtime function)
  reset(num_joints);

  // Load the differential IK plugin
  if (!parameters_.kinematics.plugin_name.empty())
  {
    try
    {
      // Make sure we destroy the interface first. Otherwise we might run into a segfault
      if (kinematics_loader_)
      {
        kinematics_.reset();
      }
      kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          parameters_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));

      if (!kinematics_->initialize(node->get_node_parameters_interface(), "kinematics"))
      {
        return controller_interface::return_type::ERROR;
      }
    }
    catch (pluginlib::PluginlibException & ex)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
        parameters_.kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::return_type::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("AdmittanceRule"),
      "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset(const size_t num_joints)
{
  // reset state message fields
  state_message_.joint_state.name.assign(num_joints, "");
  state_message_.joint_state.position.assign(num_joints, 0);
  state_message_.joint_state.velocity.assign(num_joints, 0);
  state_message_.joint_state.effort.assign(num_joints, 0);
  for (size_t i = 0; i < parameters_.joints.size(); ++i)
  {
    state_message_.joint_state.name = parameters_.joints;
  }
  state_message_.mass.data.resize(NUM_CARTESIAN_DOF, 0.0);
  state_message_.selected_axes.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.damping.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.stiffness.data.resize(NUM_CARTESIAN_DOF, 0);
  state_message_.wrench_base.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_velocity.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_acceleration.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_position.header.frame_id = parameters_.kinematics.base;
  state_message_.admittance_position.child_frame_id = "admittance_offset";

  // reset admittance state
  admittance_state_ = AdmittanceState(num_joints);

  // reset transforms and rotations
  admittance_transforms_ = AdmittanceTransforms();

  // reset forces
  wrench_world_.setZero();
  end_effector_weight_.setZero();

  // load/initialize Eigen types from parameters
  apply_parameters_update();

  return controller_interface::return_type::OK;
}

void AdmittanceRule::apply_parameters_update()
{
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
  }
  // update param values
  end_effector_weight_[2] = -parameters_.gravity_compensation.CoG.force;
  vec_to_eigen(parameters_.gravity_compensation.CoG.pos, cog_pos_);
  vec_to_eigen(parameters_.admittance.mass, admittance_state_.mass);
  vec_to_eigen(parameters_.admittance.stiffness, admittance_state_.stiffness);
  vec_to_eigen(parameters_.admittance.selected_axes, admittance_state_.selected_axes);

  for (size_t i = 0; i < NUM_CARTESIAN_DOF; ++i)
  {
    admittance_state_.mass_inv[i] = 1.0 / parameters_.admittance.mass[i];
    admittance_state_.damping[i] = parameters_.admittance.damping_ratio[i] * 2 *
                                   sqrt(admittance_state_.mass[i] * admittance_state_.stiffness[i]);
  }
}

bool AdmittanceRule::get_all_transforms(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state)
{
  // get reference transforms
  bool success = kinematics_->calculate_link_transform(
    reference_joint_state.positions, parameters_.ft_sensor.frame.id,
    admittance_transforms_.ref_base_ft_);

  // get transforms at current configuration
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.ft_sensor.frame.id, admittance_transforms_.base_ft_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.kinematics.tip, admittance_transforms_.base_tip_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.fixed_world_frame.frame.id,
    admittance_transforms_.world_base_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.gravity_compensation.frame.id,
    admittance_transforms_.base_cog_);
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions, parameters_.control.frame.id,
    admittance_transforms_.base_control_);

  return success;
}

// Update from reference joint states
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state,
  const rclcpp::Duration & period, trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  const double dt = period.seconds();

  if (parameters_.enable_parameter_update_without_reactivation)
  {
    apply_parameters_update();
  }

  bool success = get_all_transforms(current_joint_state, reference_joint_state);

  // apply filter and update wrench_world_ vector
  Eigen::Matrix<double, 3, 3> rot_world_sensor =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_ft_.rotation();
  Eigen::Matrix<double, 3, 3> rot_world_cog =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_cog_.rotation();
  process_wrench_measurements(measured_wrench, rot_world_sensor, rot_world_cog);

  // transform wrench_world_ into base frame
  admittance_state_.wrench_base.block<3, 1>(0, 0) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.block<3, 1>(0, 0);
  admittance_state_.wrench_base.block<3, 1>(3, 0) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.block<3, 1>(3, 0);

  // Compute admittance control law
  vec_to_eigen(current_joint_state.positions, admittance_state_.current_joint_pos);
  admittance_state_.rot_base_control = admittance_transforms_.base_control_.rotation();
  admittance_state_.ref_trans_base_ft = admittance_transforms_.ref_base_ft_;
  admittance_state_.ft_sensor_frame = parameters_.ft_sensor.frame.id;
  success &= calculate_admittance_rule(admittance_state_, dt);

  // if a failure occurred during any kinematics interface calls, return an error and don't
  // modify the desired reference
  if (!success)
  {
    desired_joint_state = reference_joint_state;
    return controller_interface::return_type::ERROR;
  }

  // update joint desired joint state
  for (size_t i = 0; i < num_joints_; ++i)
  {
    desired_joint_state.positions[i] =
      reference_joint_state.positions[i] + admittance_state_.joint_pos[i];
    desired_joint_state.velocities[i] =
      reference_joint_state.velocities[i] + admittance_state_.joint_vel[i];
    desired_joint_state.accelerations[i] =
      reference_joint_state.accelerations[i] + admittance_state_.joint_acc[i];
  }

  return controller_interface::return_type::OK;
}

bool AdmittanceRule::calculate_admittance_rule(AdmittanceState & admittance_state, double dt)
{
  // Create stiffness matrix in base frame. The user-provided values of admittance_state.stiffness
  // correspond to the six diagonal elements of the stiffness matrix expressed in the control frame
  auto rot_base_control = admittance_state.rot_base_control;
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> K_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> K_rot = Eigen::Matrix<double, 3, 3>::Zero();
  K_pos.diagonal() = admittance_state.stiffness.block<3, 1>(0, 0);
  K_rot.diagonal() = admittance_state.stiffness.block<3, 1>(3, 0);
  // Transform to the control frame
  // A reference is here:  https://users.wpi.edu/~jfu2/rbe502/files/force_control.pdf
  // Force Control by Luigi Villani and Joris De Schutter
  // Page 200
  K_pos = rot_base_control * K_pos * rot_base_control.transpose();
  K_rot = rot_base_control * K_rot * rot_base_control.transpose();
  K.block<3, 3>(0, 0) = K_pos;
  K.block<3, 3>(3, 3) = K_rot;

  // The same for damping
  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> D_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> D_rot = Eigen::Matrix<double, 3, 3>::Zero();
  D_pos.diagonal() = admittance_state.damping.block<3, 1>(0, 0);
  D_rot.diagonal() = admittance_state.damping.block<3, 1>(3, 0);
  D_pos = rot_base_control * D_pos * rot_base_control.transpose();
  D_rot = rot_base_control * D_rot * rot_base_control.transpose();
  D.block<3, 3>(0, 0) = D_pos;
  D.block<3, 3>(3, 3) = D_rot;

  // calculate admittance relative offset in base frame
  Eigen::Isometry3d desired_trans_base_ft;
  kinematics_->calculate_link_transform(
    admittance_state.current_joint_pos, admittance_state.ft_sensor_frame, desired_trans_base_ft);
  Eigen::Matrix<double, 6, 1> X;
  X.block<3, 1>(0, 0) =
    desired_trans_base_ft.translation() - admittance_state.ref_trans_base_ft.translation();
  auto R_ref = admittance_state.ref_trans_base_ft.rotation();
  auto R_desired = desired_trans_base_ft.rotation();
  auto R = R_desired * R_ref.transpose();
  auto angle_axis = Eigen::AngleAxisd(R);
  X.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

  // get admittance relative velocity
  auto X_dot = Eigen::Matrix<double, 6, 1>(admittance_state.admittance_velocity.data());

  // external force expressed in the base frame
  auto F_base = admittance_state.wrench_base;

  // zero out any forces in the control frame
  Eigen::Matrix<double, 6, 1> F_control;
  F_control.block<3, 1>(0, 0) = rot_base_control.transpose() * F_base.block<3, 1>(0, 0);
  F_control.block<3, 1>(3, 0) = rot_base_control.transpose() * F_base.block<3, 1>(3, 0);
  F_control = F_control.cwiseProduct(admittance_state.selected_axes);
  F_base.block<3, 1>(0, 0) = rot_base_control * F_control.block<3, 1>(0, 0);
  F_base.block<3, 1>(3, 0) = rot_base_control * F_control.block<3, 1>(3, 0);

  // Compute admittance control law in the base frame: F = M*x_ddot + D*x_dot + K*x
  Eigen::Matrix<double, 6, 1> X_ddot =
    admittance_state.mass_inv.cwiseProduct(F_base - D * X_dot - K * X);
  bool success = kinematics_->convert_cartesian_deltas_to_joint_deltas(
    admittance_state.current_joint_pos, X_ddot, admittance_state.ft_sensor_frame,
    admittance_state.joint_acc);

  // add damping if cartesian velocity falls below threshold
  for (int64_t i = 0; i < admittance_state.joint_acc.size(); ++i)
  {
    admittance_state.joint_acc[i] -=
      parameters_.admittance.joint_damping * admittance_state.joint_vel[i];
  }

  // integrate motion in joint space
  admittance_state.joint_vel += (admittance_state.joint_acc) * dt;
  admittance_state.joint_pos += admittance_state.joint_vel * dt;

  // calculate admittance velocity corresponding to joint velocity ("base_link" frame)
  success &= kinematics_->convert_joint_deltas_to_cartesian_deltas(
    admittance_state.current_joint_pos, admittance_state.joint_vel,
    admittance_state.ft_sensor_frame, admittance_state.admittance_velocity);
  success &= kinematics_->convert_joint_deltas_to_cartesian_deltas(
    admittance_state.current_joint_pos, admittance_state.joint_acc,
    admittance_state.ft_sensor_frame, admittance_state.admittance_acceleration);

  return success;
}

void AdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench,
  const Eigen::Matrix<double, 3, 3> & sensor_world_rot,
  const Eigen::Matrix<double, 3, 3> & cog_world_rot)
{
  Eigen::Matrix<double, 3, 2, Eigen::ColMajor> new_wrench;
  new_wrench(0, 0) = measured_wrench.force.x;
  new_wrench(1, 0) = measured_wrench.force.y;
  new_wrench(2, 0) = measured_wrench.force.z;
  new_wrench(0, 1) = measured_wrench.torque.x;
  new_wrench(1, 1) = measured_wrench.torque.y;
  new_wrench(2, 1) = measured_wrench.torque.z;

  // transform to world frame
  Eigen::Matrix<double, 3, 2> new_wrench_base = sensor_world_rot * new_wrench;

  // apply gravity compensation
  new_wrench_base(2, 0) -= end_effector_weight_[2];
  new_wrench_base.block<3, 1>(0, 1) -= (cog_world_rot * cog_pos_).cross(end_effector_weight_);

  // apply smoothing filter
  for (size_t i = 0; i < 6; ++i)
  {
    wrench_world_(i) = filters::exponentialSmoothing(
      new_wrench_base(i), wrench_world_(i), parameters_.ft_sensor.filter_coefficient);
  }
}

const control_msgs::msg::AdmittanceControllerState & AdmittanceRule::get_controller_state()
{
  for (size_t i = 0; i < NUM_CARTESIAN_DOF; ++i)
  {
    state_message_.stiffness.data[i] = admittance_state_.stiffness[i];
    state_message_.damping.data[i] = admittance_state_.damping[i];
    state_message_.selected_axes.data[i] = static_cast<bool>(admittance_state_.selected_axes[i]);
    state_message_.mass.data[i] = admittance_state_.mass[i];
  }

  for (size_t i = 0; i < parameters_.joints.size(); ++i)
  {
    state_message_.joint_state.name[i] = parameters_.joints[i];
    state_message_.joint_state.position[i] = admittance_state_.joint_pos[i];
    state_message_.joint_state.velocity[i] = admittance_state_.joint_vel[i];
    state_message_.joint_state.effort[i] = admittance_state_.joint_acc[i];
  }

  state_message_.wrench_base.wrench.force.x = admittance_state_.wrench_base[0];
  state_message_.wrench_base.wrench.force.y = admittance_state_.wrench_base[1];
  state_message_.wrench_base.wrench.force.z = admittance_state_.wrench_base[2];
  state_message_.wrench_base.wrench.torque.x = admittance_state_.wrench_base[3];
  state_message_.wrench_base.wrench.torque.y = admittance_state_.wrench_base[4];
  state_message_.wrench_base.wrench.torque.z = admittance_state_.wrench_base[5];

  state_message_.admittance_velocity.twist.linear.x = admittance_state_.admittance_velocity[0];
  state_message_.admittance_velocity.twist.linear.y = admittance_state_.admittance_velocity[1];
  state_message_.admittance_velocity.twist.linear.z = admittance_state_.admittance_velocity[2];
  state_message_.admittance_velocity.twist.angular.x = admittance_state_.admittance_velocity[3];
  state_message_.admittance_velocity.twist.angular.y = admittance_state_.admittance_velocity[4];
  state_message_.admittance_velocity.twist.angular.z = admittance_state_.admittance_velocity[5];

  state_message_.admittance_acceleration.twist.linear.x =
    admittance_state_.admittance_acceleration[0];
  state_message_.admittance_acceleration.twist.linear.y =
    admittance_state_.admittance_acceleration[1];
  state_message_.admittance_acceleration.twist.linear.z =
    admittance_state_.admittance_acceleration[2];
  state_message_.admittance_acceleration.twist.angular.x =
    admittance_state_.admittance_acceleration[3];
  state_message_.admittance_acceleration.twist.angular.y =
    admittance_state_.admittance_acceleration[4];
  state_message_.admittance_acceleration.twist.angular.z =
    admittance_state_.admittance_acceleration[5];

  state_message_.admittance_position = tf2::eigenToTransform(admittance_state_.admittance_position);

  state_message_.ref_trans_base_ft.header.frame_id = parameters_.kinematics.base;
  state_message_.ref_trans_base_ft.header.frame_id = "ft_reference";
  state_message_.ref_trans_base_ft = tf2::eigenToTransform(admittance_state_.ref_trans_base_ft);

  Eigen::Quaterniond quat(admittance_state_.rot_base_control);
  state_message_.rot_base_control.w = quat.w();
  state_message_.rot_base_control.x = quat.x();
  state_message_.rot_base_control.y = quat.y();
  state_message_.rot_base_control.z = quat.z();

  state_message_.ft_sensor_frame.data =
    admittance_state_.ft_sensor_frame;  // TODO(anyone) remove dynamic allocation here

  return state_message_;
}

template <typename T1, typename T2>
void AdmittanceRule::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++)
  {
    for (auto row = 0; row < matrix.rows(); row++)
    {
      matrix(row, col) = data[row + col * matrix.rows()];
    }
  }
}

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
