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
        kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass >>(
            parameters_.kinematics.plugin_package, "kinematics_interface::KinematicsBaseClass");
        kinematics_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
            kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));
        if (!kinematics_->initialize(node, parameters_.kinematics.tip)) {
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

    //allocate dynamic buffers
    joint_buffer_vec_.assign(num_joints, 0.0);
    transform_buffer_vec_.assign(16, 0.0);
    cart_buffer_vec_.assign(6, 0.0);

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
    rotations_ = Rotations();

    // reset forces
    wrench_world_.setZero();
    measured_wrench_.setZero();
    ee_weight.setZero();

    return controller_interface::return_type::OK;
  }

  void AdmittanceRule::apply_parameters_update(){
    if (parameter_handler_->is_old(parameters_)){
      parameters_ = parameter_handler_->get_params();
    }
    // update param values
    ee_weight[2] = -parameters_.gravity_compensation.CoG.force;
    vec_to_eigen(parameters_.gravity_compensation.CoG.pos, cog_);
    vec_to_eigen(parameters_.admittance.mass, admittance_state_.mass_);
    vec_to_eigen(parameters_.admittance.stiffness, admittance_state_.stiffness_);
    vec_to_eigen(parameters_.admittance.selected_axes, admittance_state_.selected_axes_);

    for (auto i = 0ul; i < 6; ++i) {
      admittance_state_.mass_inv_[i] = 1.0 / parameters_.admittance.mass[i];
      admittance_state_.damping_[i] = parameters_.admittance.damping_ratio[i] * 2 * sqrt(admittance_state_.mass_[i] * admittance_state_.stiffness_[i]);
    }

  }

  bool AdmittanceRule::get_all_transforms(const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
                                      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state){
    // get all reference transforms
    bool success = get_transform(reference_joint_state.positions, parameters_.ft_sensor.frame.id,
                             parameters_.ft_sensor.frame.external, trans_ref_.base_ft_);
    success &= get_transform(reference_joint_state.positions, parameters_.kinematics.tip,
                             false, trans_ref_.base_tip_);
    success &= get_transform(reference_joint_state.positions, parameters_.fixed_world_frame.frame.id,
                             parameters_.fixed_world_frame.frame.external, trans_ref_.world_base_);
    success &= get_transform(reference_joint_state.positions, parameters_.ft_sensor.frame.id,
                             parameters_.ft_sensor.frame.external, trans_ref_.base_sensor_);
    success &= get_transform(reference_joint_state.positions, parameters_.gravity_compensation.frame.id,
                             parameters_.gravity_compensation.frame.external, trans_ref_.base_cog_);

    // get all current transforms
    success &= get_transform(current_joint_state.positions, parameters_.ft_sensor.frame.id,
                                  parameters_.ft_sensor.frame.external, trans_.base_ft_);
    success &= get_transform(current_joint_state.positions, parameters_.kinematics.tip,
                                  false, trans_.base_tip_);
    success &= get_transform(current_joint_state.positions, parameters_.fixed_world_frame.frame.id,
                                     parameters_.fixed_world_frame.frame.external, trans_.world_base_);
    success &= get_transform(current_joint_state.positions, parameters_.ft_sensor.frame.id,
                                      parameters_.ft_sensor.frame.external, trans_.base_sensor_);
    success &= get_transform(current_joint_state.positions, parameters_.gravity_compensation.frame.id,
                                   parameters_.gravity_compensation.frame.external, trans_.base_cog_);
    rotations_.base_control_ = trans_.base_control_.block<3,3>(0,0);
    rotations_.base_ft_ = trans_.base_ft_.block<3,3>(0,0);
    rotations_.base_tip_ = trans_.base_tip_.block<3,3>(0,0);
    rotations_.world_base_ = trans_.world_base_.block<3,3>(0,0);
    rotations_.base_world_ = rotations_.world_base_.transpose();
    rotations_.base_sensor_ = trans_.base_sensor_.block<3,3>(0,0);
    rotations_.base_cog_ = trans_.base_cog_.block<3,3>(0,0);
    rotations_.world_sensor_ = rotations_.world_base_ * rotations_.base_sensor_;
    rotations_.world_cog_ = rotations_.world_base_ * rotations_.base_cog_;

    return success;
  }

  // Update from reference joint states
  controller_interface::return_type AdmittanceRule::update(
      const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
      const geometry_msgs::msg::Wrench &measured_wrench,
      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
      const rclcpp::Duration &period,
      trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_state) {

    double dt = period.seconds() + ((double) period.nanoseconds()) * 1E-9;

    if (parameters_.enable_parameter_update_without_reactivation){
      apply_parameters_update();
    }

    bool success = get_all_transforms(current_joint_state, reference_joint_state);

    // apply filter and update wrench_world_ vector
    process_wrench_measurements(measured_wrench, rotations_.world_sensor_, rotations_.world_cog_);

    // transform wrench_world_ into base frame
    admittance_state_.wrench_base = rotations_.base_world_ * wrench_world_;

    // calculate feedforward cartesian velocity in control frame
    success &= convert_joint_deltas_to_cartesian_deltas(reference_joint_state.positions,
                                                       reference_joint_state.velocities,
                                                       parameters_.ft_sensor.frame.id,
                                                        admittance_state_.feedforward_vel_);
    admittance_state_.feedforward_vel_ *= parameters_.use_feedforward_commanded_input;

    // Compute admittance control law
    success &= calculate_admittance_rule(current_joint_state.positions, rotations_.base_control_, trans_.base_ft_,
                              trans_ref_.base_ft_, parameters_.ft_sensor.frame.id, dt, admittance_state_);

    // if a failure occurred during any kinematics interface calls, return an error and don't modify the desired reference
    if (!success) {
      desired_joint_state = reference_joint_state;
      return controller_interface::return_type::ERROR;
    }

    // update joint desired joint state
    for (auto i = 0ul; i < reference_joint_state.positions.size(); i++) {
      desired_joint_state.positions[i] = reference_joint_state.positions[i] + admittance_state_.joint_pos_[i];
      state_message_.error_joint_state.positions[i] =
          reference_joint_state.positions[i] - current_joint_state.positions[i];
    }
    for (auto i = 0ul; i < reference_joint_state.velocities.size(); i++) {
      desired_joint_state.velocities[i] = admittance_state_.joint_vel_[i];
      state_message_.error_joint_state.velocities[i] = reference_joint_state.velocities[i] - current_joint_state.velocities[i];
    }
    for (auto i = 0ul; i < reference_joint_state.accelerations.size(); i++) {
      desired_joint_state.accelerations[i] = admittance_state_.joint_acc_[i];
    }

    // update admittance state message
    state_message_.actual_joint_state = current_joint_state;
    state_message_.desired_joint_state = desired_joint_state;

    return controller_interface::return_type::OK;
  }

  bool AdmittanceRule::calculate_admittance_rule(const std::vector<double>& current_joint_positions,
                                                 const Eigen::Matrix<double,3,3,Eigen::ColMajor>& base_control,
                                                 Eigen::Matrix<double,4,4,Eigen::ColMajor> base_ft,
                                                 Eigen::Matrix<double,4,4,Eigen::ColMajor> ref_base_ft,
                                                 const std::string & ft_sensor_frame,
                                                 double dt,
                                                 AdmittanceState &admittance_state) {

    // reshape inputs
    auto admittance_velocity_flat = Eigen::Matrix<double, 6, 1>(admittance_state.admittance_velocity_.data());
    auto wrench_flat = Eigen::Matrix<double, 6, 1>(admittance_state.wrench_base.data());
    const auto desired_vel_flat = Eigen::Matrix<double, 6, 1>(admittance_state.feedforward_vel_.data());

    // create stiffness matrix
    Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> K_pos = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 3> K_rot = Eigen::Matrix<double, 3, 3>::Zero();
    K_pos.diagonal() = admittance_state.stiffness_.block<3,1>(0,0);
    K_rot.diagonal() = admittance_state.stiffness_.block<3,1>(3,0);
    K_pos = base_control.transpose()*K_pos*base_control;
    K_rot = base_control.transpose()*K_rot*base_control;
    K.block<3,3>(0,0) = K_pos;
    K.block<3,3>(3,3) = K_rot;

    // create damping matrix
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> D_pos = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 3> D_rot = Eigen::Matrix<double, 3, 3>::Zero();
    D_pos.diagonal() = admittance_state.damping_.block<3,1>(0,0);
    D_rot.diagonal() = admittance_state.damping_.block<3,1>(3,0);
    D_pos = base_control.transpose()*D_pos*base_control;
    D_rot = base_control.transpose()*D_rot*base_control;
    D.block<3,3>(0,0) = D_pos;
    D.block<3,3>(3,3) = D_rot;

    // calculate pose error
    Eigen::Matrix<double, 6, 1> pose_error;
    pose_error.block<3, 1>(0, 0) = -admittance_state.admittance_position_.block<3, 1>(0, 3);

    // calculate rotation error
    Eigen::Matrix<double, 3, 3> admittance_rotation = admittance_state.admittance_position_.block<3, 3>(0, 0);
    Eigen::Vector3d V = get_rotation_axis(admittance_rotation);
    double theta = acos(std::clamp((1.0 / 2.0) * (admittance_rotation.trace() - 1), -1.0, 1.0));
    pose_error.block<3, 1>(3, 0) = theta * V;

    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
    Eigen::Matrix<double, 6, 1> admittance_acceleration_flat = admittance_state.mass_inv_.cwiseProduct(wrench_flat +
                                      D*(desired_vel_flat-admittance_velocity_flat) + K*pose_error);

    // reshape admittance outputs
    admittance_state.admittance_acceleration_ = Eigen::Matrix<double, 3, 2>(admittance_acceleration_flat.data());

    // integrate velocity
    admittance_state.admittance_velocity_ += admittance_state.admittance_acceleration_*dt;
    admittance_state.admittance_position_.block<3, 1>(0, 3) += admittance_state.admittance_velocity_.block<3,1>(0,0)*dt;

    // integrate rotational velocity
    Eigen::Matrix3d skew_symmetric;
    auto rot_vel = admittance_state.admittance_velocity_;
    skew_symmetric << 0, -rot_vel(2, 1), rot_vel(1, 1),
        rot_vel(2, 1), 0, -rot_vel(0, 1),
        -rot_vel(1, 1), rot_vel(0, 1), 0;
    Eigen::Matrix3d R_dot = skew_symmetric * admittance_rotation;
    admittance_rotation += R_dot * dt;
    normalize_rotation(admittance_rotation);
    admittance_state.admittance_position_.block<3, 3>(0, 0) = admittance_rotation;

    // calculate position drift due to integrating the joint positions
    auto admittance_position_base = ref_base_ft.block<3,1>(0, 3) + admittance_state.admittance_position_.block<3,1>(0, 3);
    Eigen::Matrix<double, 3, 1> admittance_cur_position_base = base_ft.block<3,1>(0,3);
    Eigen::Matrix<double, 3, 3> ft_rot = base_ft.block<3,3>(0,0);
    Eigen::Matrix<double, 3, 3> reference_ft_rot = ref_base_ft.block<3,3>(0,0);

    // calculate rotation drift due to integrating the joint positions
    Eigen::Matrix<double, 3, 3> admittance_rot = admittance_state.admittance_position_.block<3,3>(0,0);
    Eigen::Matrix<double, 3, 3> R = admittance_rot.transpose()*(reference_ft_rot.transpose()*reference_ft_rot);

    // calculate correction term
    Eigen::Matrix<double, 3, 2> correction_velocity;
    correction_velocity.block<3, 1>(0, 0) = (admittance_position_base - admittance_cur_position_base);
    V = get_rotation_axis(R);
    theta = acos(std::clamp((1.0 / 2.0) * (R.trace() - 1), -1.0, 1.0));
    correction_velocity.block<3, 1>(0, 1) = .1*theta*V;

    // calculate joint velocities that correspondence to cartesian velocities
    bool success = convert_cartesian_deltas_to_joint_deltas(current_joint_positions, admittance_state.admittance_velocity_ + correction_velocity,
                                             ft_sensor_frame,
                                             admittance_state.joint_vel_);
    // calculate joint velocities that correspondence to cartesian accelerations
    success &= convert_cartesian_deltas_to_joint_deltas(current_joint_positions, admittance_state.admittance_acceleration_,
                                             ft_sensor_frame,
                                             admittance_state.joint_acc_);

    for (auto i = 0ul; i < admittance_state.joint_pos_.size(); i++) {
      admittance_state.joint_pos_[i] += admittance_state.joint_vel_[i]*dt;
    }

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

    // store value for state message
    measured_wrench_ = new_wrench_base;

    // apply gravity compensation
    new_wrench_base(2, 0) -= ee_weight[2];
    new_wrench_base.block<3, 1>(0, 1) -= (cog_world_rot * cog_).cross(ee_weight);

    // apply smoothing filter
    for (auto i = 0; i < 6; i++) {
      wrench_world_(i) = filters::exponentialSmoothing(
          new_wrench_base(i), wrench_world_(i), parameters_.ft_sensor.filter_coefficient);
    }

  }

  Eigen::Vector3d AdmittanceRule::get_rotation_axis(const Eigen::Matrix3d &R) const {

    Eigen::Vector3d V;
    bool solved = false;
    double v1 = 1;
    double v2, v3;
    double R_buffer[9];

    // rotate matrix rows and columns to hit every edge case
    for (int i = 0; i < 3; i++) {

      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          R_buffer[((c * 3 - 3 * i) % 9) + ((r - i) % 3)] = R(r, c);
        }
      }
      auto R11 = R_buffer[0];
      auto R12 = R_buffer[1];
      auto R32 = R_buffer[7];
      auto R13 = R_buffer[2];
      auto R33 = R_buffer[8];
      // degenerate: one axis rotation
      if (abs(R12 + R13) < utility::ROT_AXIS_EPSILON && R11 > 0) {
        v2 = 0;
        v3 = 0;
        V[i % 3] = v1;
        V[(i + 1) % 3] = v2;
        V[(i + 2) % 3] = v3;
        solved = true;
        break;
      }
      // degenerate: two axis rotation
      if (abs(R12 + R13) < utility::ROT_AXIS_EPSILON && R11 < 0) {
        v1 = 0;
        v2 = 1;
        v3 = R32 / (1 - R33);
        V[i % 3] = v1;
        V[(i + 1) % 3] = v2;
        V[(i + 2) % 3] = v3;
        solved = true;
        break;
      }
    }

    // general case: three axis rotation
    if (!solved) {
      v3 = (-R(0, 1) - ((R(1, 1) - 1) * (1 - R(0, 0))) / R(1, 0));
      // if v3 is zero, special case
      if (abs(v3) > utility::ROT_AXIS_EPSILON) {
        v3 = v3 / (R(2, 1) - ((R(1, 1) - 1) * (R(2, 0))) / (R(1, 0)));
      }
      v2 = (1 - R(0, 0) - R(2, 0) * v3) / R(1, 0);

      V[0] = v1;
      V[1] = v2;
      V[2] = v3;
    }

    V.normalize();

    // if trace of the rotation matrix derivative is negative, then rotation axis needs to be flipped
    auto tmp = V[0] * (R(1, 2) - R(2, 1)) + V[1] * (R(2, 0) - R(0, 2))
               + V[2] * (R(0, 1) - R(1, 0));
    double sign = (tmp >= 0) ? 1.0 : -1.0;

    return sign*V;
  }

  void AdmittanceRule::normalize_rotation(Eigen::Matrix3d &R) {
    // enforce orthonormal constraint

    Eigen::Vector3d R_0 = R.block<3, 1>(0, 0);
    R_0.normalize();
    Eigen::Vector3d R_1 = R.block<3, 1>(0, 1);
    R_1.normalize();
    Eigen::Vector3d R_2 = R.block<3, 1>(0, 2);
    R_2.normalize();

    double drift = R_0.dot(R_1);
    R_1 += R_0 * (-drift);
    R_1.normalize();

    R_2 = R_0.cross(R_1);
    R_2.normalize();

    Eigen::Matrix3d R_out;
    R.col(0) << R_0;
    R.col(1) << R_1;
    R.col(2) << R_2;

  }

  bool
  AdmittanceRule::get_transform(const std::vector<double> &positions, const std::string &link_name, bool external, Eigen::Matrix<double,4,4,Eigen::ColMajor>& transform) {
    bool success;
     if (external) {
      transform.setIdentity();
      try {
        auto transform_msg = tf_buffer_->lookupTransform(parameters_.kinematics.base, link_name, tf2::TimePointZero);
        transform.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>(tf2::transformToKDL(transform_msg).M.data);
        transform(0, 3) = transform_msg.transform.translation.x;
        transform(1, 3) = transform_msg.transform.translation.y;
        transform(2, 3) = transform_msg.transform.translation.z;
      } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(
            rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
        success = false;
      }
    } else {
       success = kinematics_->calculate_link_transform(positions, link_name, transform_buffer_vec_);
      vec_to_eigen(transform_buffer_vec_, transform);
    }
    return success;
  }

  bool AdmittanceRule::convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &positions,
                                                                const Eigen::Matrix<double, 3, 2> &cartesian_delta,
                                                                const std::string &link_name,
                                                                Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_delta) {
    memcpy(cart_buffer_vec_.data(), cartesian_delta.data(), 6 * sizeof(double));
    bool success = kinematics_->convert_cartesian_deltas_to_joint_deltas(positions, cart_buffer_vec_, link_name,
                                                                     joint_buffer_vec_);
    joint_delta = Eigen::Map<Eigen::Matrix<double, 6, 1>>(joint_buffer_vec_.data(), 6, 1);
    return success;
  }

  bool AdmittanceRule::convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &positions,
                                                           const std::vector<double> &joint_delta,
                                                           const std::string &link_name,
                                                           Eigen::Matrix<double, 3, 2>& cartesian_delta) {

    bool success = kinematics_->convert_joint_deltas_to_cartesian_deltas(positions, joint_delta, link_name,
                                                                     cart_buffer_vec_);
    vec_to_eigen(cart_buffer_vec_, cartesian_delta);
    return success;
  }

  void AdmittanceRule::get_controller_state(control_msgs::msg::AdmittanceControllerState &state_message) {

    // TODO these fields are not used
    eigen_to_msg(0*wrench_world_, parameters_.control.frame.id, state_message.input_wrench_command);
    state_message.input_pose_command.header.frame_id = parameters_.control.frame.id;

    eigen_to_msg(wrench_world_, parameters_.fixed_world_frame.frame.id, state_message.measured_wrench_filtered);
    eigen_to_msg(measured_wrench_, parameters_.fixed_world_frame.frame.id, state_message.measured_wrench);
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

  void AdmittanceRule::eigen_to_msg(const Eigen::Matrix<double, 3, 2> &wrench, const std::string &frame_id,
                                    geometry_msgs::msg::WrenchStamped &wrench_msg) {
    wrench_msg.wrench.force.x = wrench(0, 0);
    wrench_msg.wrench.force.y = wrench(1, 0);
    wrench_msg.wrench.force.z = wrench(2, 0);
    wrench_msg.wrench.torque.x = wrench(0, 1);
    wrench_msg.wrench.torque.y = wrench(1, 1);
    wrench_msg.wrench.torque.z = wrench(2, 1);
    wrench_msg.header.frame_id = frame_id;
    wrench_msg.header.stamp = clock_->now();
  }

  template<typename T1, typename T2>
  void AdmittanceRule::vec_to_eigen(const std::vector<T1>& data, T2 &matrix) {
    for (auto col = 0; col < matrix.cols(); col++) {
      for (auto row = 0; row < matrix.rows(); row++) {
        matrix(row, col) = data[row + col*matrix.rows()];
      }
    }

  }

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
