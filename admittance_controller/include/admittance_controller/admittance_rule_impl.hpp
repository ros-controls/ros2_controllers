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

#include <tf2_ros/transform_listener.h>
#include "admittance_controller/admittance_rule.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"

namespace admittance_controller {

  controller_interface::return_type
  AdmittanceRule::configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, int num_joints) {
    // configure admittance rule using num_joints and load kinematics interface

    num_joints_ = num_joints;

    // initialize ROS functions
    clock_ = node->get_clock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize memory and values to zero  (non-realtime function)
    reset(num_joints);

    // Load the differential IK plugin
    if (!parameters_.kinematics.plugin_name.empty()) {
      try {
        kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterfaceBase >>(
            parameters_.kinematics.plugin_package, "kinematics_interface::KinematicsInterfaceBase");
        kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterfaceBase>(
            kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));
        if (!kinematics_->initialize(node->get_node_parameters_interface(), parameters_.kinematics.tip)) {
          return controller_interface::return_type::ERROR;
        }
      }
      catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
                     parameters_.kinematics.plugin_name.c_str(), ex.what());
        return controller_interface::return_type::ERROR;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                   "A differential IK plugin name was not specified in the config file.");
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type AdmittanceRule::reset(int num_joints) {
    // reset all values back to default

    // reset state message fields
    state_message_.error_joint_state.positions.resize(num_joints, 0.0);
    state_message_.error_joint_state.velocities.resize(num_joints, 0.0);
    state_message_.admittance_rule_calculated_values.positions.resize(6, 0.0);
    state_message_.admittance_rule_calculated_values.velocities.resize(6, 0.0);
    state_message_.admittance_rule_calculated_values.accelerations.resize(6, 0.0);

    // reset admittance state
    admittance_state_ = AdmittanceState(num_joints);

    // reset transforms and rotations
    trans_ = Transforms();
    trans_ref_ = Transforms();

    // reset forces
    wrench_world_.setZero();
    ee_weight.setZero();

    // load/initialize Eigen types from parameters
    apply_parameters_update();

    return controller_interface::return_type::OK;
  }

  void AdmittanceRule::apply_parameters_update() {
    if (parameter_handler_->is_old(parameters_)) {
      parameters_ = parameter_handler_->get_params();
    }
    // update param values
    ee_weight[2] = -parameters_.gravity_compensation.CoG.force;
    vec_to_eigen(parameters_.gravity_compensation.CoG.pos, cog_);
    vec_to_eigen(parameters_.admittance.mass, admittance_state_.mass);
    vec_to_eigen(parameters_.admittance.stiffness, admittance_state_.stiffness);
    vec_to_eigen(parameters_.admittance.selected_axes, admittance_state_.selected_axes);

    for (auto i = 0ul; i < 6; ++i) {
      admittance_state_.mass_inv[i] = 1.0 / parameters_.admittance.mass[i];
      admittance_state_.damping[i] = parameters_.admittance.damping_ratio[i] * 2 *
                                      sqrt(admittance_state_.mass[i] * admittance_state_.stiffness[i]);
    }

  }

  bool AdmittanceRule::get_all_transforms(const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
                                          const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
                                          const AdmittanceState &admittance_state) {
    // get all reference transforms
    bool success = kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.ft_sensor.frame.id, trans_ref_.base_ft_);
    success &= kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.kinematics.tip,trans_ref_.base_tip_);
    success &= kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.fixed_world_frame.frame.id,trans_ref_.world_base_);
    success &= kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.ft_sensor.frame.id,trans_ref_.base_sensor_);
    success &= kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.gravity_compensation.frame.id,trans_ref_.base_cog_);
    success &= kinematics_->calculate_link_transform(reference_joint_state.positions, parameters_.control.frame.id,trans_ref_.base_control_);

    // get all current transforms
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.ft_sensor.frame.id,trans_.base_ft_);
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.kinematics.tip,trans_.base_tip_);
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.fixed_world_frame.frame.id,trans_.world_base_);
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.ft_sensor.frame.id,trans_.base_sensor_);
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.gravity_compensation.frame.id,trans_.base_cog_);
    success &= kinematics_->calculate_link_transform(current_joint_state.positions, parameters_.control.frame.id,trans_.base_control_);

    return success;
  }

  // Update from reference joint states
  controller_interface::return_type AdmittanceRule::update(
      const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
      const geometry_msgs::msg::Wrench &measured_wrench,
      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
      const rclcpp::Duration &period,
      trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_state) {

    double dt = period.seconds();

    if (parameters_.enable_parameter_update_without_reactivation) {
      apply_parameters_update();
    }

    bool success = get_all_transforms(current_joint_state, reference_joint_state, admittance_state_);

    // calculate needed rotations
    Eigen::Matrix<double, 3, 3> rot_base_sensor = trans_.base_sensor_.rotation();
    Eigen::Matrix<double, 3, 3> rot_world_base = trans_.world_base_.rotation();
    Eigen::Matrix<double, 3, 3> rot_base_cog = trans_.base_cog_.rotation();
    Eigen::Matrix<double, 3, 3> rot_base_control = trans_.base_control_.rotation();

    // apply filter and update wrench_world_ vector
    Eigen::Matrix<double, 3, 3> rot_world_sensor = rot_world_base*rot_base_sensor;
    Eigen::Matrix<double, 3, 3> rot_world_cog = rot_world_base*rot_base_cog;
    process_wrench_measurements(measured_wrench, rot_world_sensor, rot_world_cog);

    // transform wrench_world_ into base frame
    admittance_state_.wrench_base.block<3,1>(0,0) = rot_world_base.transpose() * wrench_world_.block<3,1>(0,0);
    admittance_state_.wrench_base.block<3,1>(3,0) = rot_world_base.transpose() * wrench_world_.block<3,1>(3,0);

    // Compute admittance control law
    vec_to_eigen(current_joint_state.positions, admittance_state_.current_joint_pos);
    admittance_state_.rot_base_control = rot_base_control;
    admittance_state_.ref_trans_base_ft = trans_ref_.base_ft_;
    admittance_state_.ft_sensor_frame = parameters_.ft_sensor.frame.id;
    success &= calculate_admittance_rule(admittance_state_, dt);

    // if a failure occurred during any kinematics interface calls, return an error and don't modify the desired reference
    if (!success) {
      desired_joint_state = reference_joint_state;
      return controller_interface::return_type::ERROR;
    }

    // update joint desired joint state
    for (auto i = 0ul; i < reference_joint_state.positions.size(); i++) {
      desired_joint_state.positions[i] = reference_joint_state.positions[i] + admittance_state_.joint_pos[i];
      state_message_.error_joint_state.positions[i] =
          reference_joint_state.positions[i] - current_joint_state.positions[i];
    }
    for (auto i = 0ul; i < reference_joint_state.velocities.size(); i++) {
      desired_joint_state.velocities[i] = admittance_state_.joint_vel[i];
      state_message_.error_joint_state.velocities[i] =
          reference_joint_state.velocities[i] - current_joint_state.velocities[i];
    }
    for (auto i = 0ul; i < reference_joint_state.accelerations.size(); i++) {
      desired_joint_state.accelerations[i] = admittance_state_.joint_acc[i];
    }

    // update admittance state message
    state_message_.actual_joint_state = current_joint_state;
    state_message_.desired_joint_state = desired_joint_state;

    return controller_interface::return_type::OK;
  }

  bool AdmittanceRule::calculate_admittance_rule(AdmittanceState &admittance_state, double dt) {

    // create stiffness matrix
    auto rot_base_control = admittance_state.rot_base_control;
    Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> K_pos = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 3> K_rot = Eigen::Matrix<double, 3, 3>::Zero();
    K_pos.diagonal() = admittance_state.stiffness.block<3, 1>(0, 0);
    K_rot.diagonal() = admittance_state.stiffness.block<3, 1>(3, 0);
    K_pos = rot_base_control * K_pos * rot_base_control.transpose();
    K_rot = rot_base_control * K_rot * rot_base_control.transpose();
    K.block<3, 3>(0, 0) = K_pos;
    K.block<3, 3>(3, 3) = K_rot;

    // create damping matrix
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> D_pos = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 3> D_rot = Eigen::Matrix<double, 3, 3>::Zero();
    D_pos.diagonal() = admittance_state.damping.block<3, 1>(0, 0);
    D_rot.diagonal() = admittance_state.damping.block<3, 1>(3, 0);
    D_pos = rot_base_control * D_pos * rot_base_control.transpose();
    D_rot = rot_base_control * D_rot * rot_base_control.transpose();
    D.block<3, 3>(0, 0) = D_pos;
    D.block<3, 3>(3, 3) = D_rot;

    // calculate admittance relative offset
    Eigen::Isometry3d desired_trans_base_ft;
    kinematics_->calculate_link_transform(admittance_state.current_joint_pos, admittance_state.ft_sensor_frame, desired_trans_base_ft);
    Eigen::Matrix<double, 6, 1> X;
    X.block<3, 1>(0, 0) = desired_trans_base_ft.translation() - admittance_state.ref_trans_base_ft.translation();
    auto R_ref = admittance_state.ref_trans_base_ft.rotation();
    auto R_desired = desired_trans_base_ft.rotation();
    auto R = R_desired * R_ref.transpose();
    auto angle_axis = Eigen::AngleAxisd(R);
    X.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

    // get admittance relative velocity
    auto X_dot = Eigen::Matrix<double, 6, 1>(admittance_state.admittance_velocity.data());

    // get external force
    auto F = Eigen::Matrix<double, 6, 1>(admittance_state.wrench_base.data());

    // Compute admittance control law: F = M*a + D*v + S*x
    Eigen::Matrix<double, 6, 1> X_ddot = admittance_state.mass_inv.cwiseProduct(F - D * X_dot - K * X);
    bool success = kinematics_->convert_cartesian_deltas_to_joint_deltas(admittance_state.current_joint_pos, X_ddot,
                                                                         admittance_state.ft_sensor_frame, admittance_state.joint_acc);

    // integrate motion in joint space
    admittance_state.joint_vel += admittance_state.joint_acc * dt;
    admittance_state.joint_pos += admittance_state.joint_vel * dt;

    // calculate admittance velocity corresponding to joint velocity
    success &= kinematics_->convert_joint_deltas_to_cartesian_deltas(admittance_state.current_joint_pos, admittance_state.joint_vel, admittance_state.ft_sensor_frame,
                                                        admittance_state.admittance_velocity);

    return success;
  }

  void AdmittanceRule::process_wrench_measurements(
      const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 3, 3> &sensor_world_rot,
      const Eigen::Matrix<double, 3, 3> &cog_world_rot
  ) {
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
    new_wrench_base(2, 0) -= ee_weight[2];
    new_wrench_base.block<3, 1>(0, 1) -= (cog_world_rot * cog_).cross(ee_weight);

    // apply smoothing filter
    for (auto i = 0; i < 6; i++) {
      wrench_world_(i) = filters::exponentialSmoothing(
          new_wrench_base(i), wrench_world_(i), parameters_.ft_sensor.filter_coefficient);
    }

  }

  void AdmittanceRule::get_controller_state(control_msgs::msg::AdmittanceControllerState &state_message) {

    // TODO these fields are not used
//    eigen_to_msg(0 * wrench_world_, parameters_.control.frame.id, state_message.input_wrench_command);
//    state_message.input_pose_command.header.frame_id = parameters_.control.frame.id;
//
//    eigen_to_msg(wrench_world_, parameters_.fixed_world_frame.frame.id, state_message.measured_wrench_filtered);
//    eigen_to_msg(measured_wrench_, parameters_.fixed_world_frame.frame.id, state_message.measured_wrench);
//    eigen_to_msg(control_rot_.transpose() * world_rot_.transpose() * measured_wrench_, parameters_.control_.frame_.id_,
//                 state_message.measured_wrench_control_frame);
//    eigen_to_msg(ee_rot_.transpose() * world_rot_.transpose() * measured_wrench_, parameters_.kinematics_.tip_,
//                 state_message.measured_wrench_endeffector_frame);

    state_message.joint_names = parameters_.joints;
    state_message.desired_joint_state = state_message_.desired_joint_state;
    state_message.actual_joint_state = state_message_.actual_joint_state;
    state_message.error_joint_state = state_message_.error_joint_state;


    state_message.admittance_rule_calculated_values = state_message_.admittance_rule_calculated_values;

  }

  template<typename T1, typename T2>
  void AdmittanceRule::vec_to_eigen(const std::vector<T1> &data, T2 &matrix) {
    for (auto col = 0; col < matrix.cols(); col++) {
      for (auto row = 0; row < matrix.rows(); row++) {
        matrix(row, col) = data[row + col * matrix.rows()];
      }
    }
  }

  template<typename T1, typename T2>
  void AdmittanceRule::eigen_to_vec(const T2 &matrix, std::vector<T1> &data) {
    for (auto col = 0; col < matrix.cols(); col++) {
      for (auto row = 0; row < matrix.rows(); row++) {
        data[row + col * matrix.rows()] = matrix(row, col);
      }
    }
  }


}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
