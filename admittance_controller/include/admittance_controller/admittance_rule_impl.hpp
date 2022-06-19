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
/// \authors: Denis Stogl, Andy Zelenak

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_

#include "admittance_controller/admittance_rule.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace admittance_controller {

  controller_interface::return_type
  AdmittanceRule::configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int num_joints) {
    // initialize memory  (non-realtime function)

    num_joints_ = num_joints;

    // initialize all value to zero
    reset();

    // Load the differential IK plugin
    if (!parameters_.ik_plugin_name_.empty()) {
      try {
        // TODO(destogl): add "ik_interface" into separate package and then rename the package in
        // the next line from "admittance_controller" to "ik_base_plugin"
        ik_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>>(
            "kdl_plugin", "kinematics_interface::KinematicsBaseClass");
        ik_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
            ik_loader_->createUnmanagedInstance(parameters_.ik_plugin_name_));
        if (!ik_->initialize(node, parameters_.end_effector_name_)) {
          return controller_interface::return_type::ERROR;
        }
      }
      catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
                     parameters_.ik_plugin_name_.c_str(), ex.what());
        return controller_interface::return_type::ERROR;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                   "A differential IK plugin name was not specified in the config file.");
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type AdmittanceRule::reset() {
    // reset all values back to zero

    // admittance state vectors
    admittance_position_vec.resize(16, 0.0);
    admittance_velocity_vec.resize(6, 0.0);
    admittance_acceleration_vec.resize(6, 0.0);

    // admittance state vectors in joint space
    joint_pos.resize(num_joints_, 0.0);
    joint_vel.resize(num_joints_, 0.0);
    joint_acc.resize(num_joints_, 0.0);

    // link transforms in base link frame
    cur_ee_transform_vec.resize(16, 0.0);
    desired_ee_transform_vec.resize(16, 0.0);
    cur_sensor_transform_vec.resize(16, 0.0);
    cur_control_transform_vec.resize(16, 0.0);

    cur_ee_transform.setZero();
    desired_ee_transform.setZero();
    cur_sensor_transform.setZero();
    cur_control_transform.setZero();
    admittance_position_.setIdentity();
    admittance_velocity_.setZero();
    admittance_acceleration_.setZero();
    wrench.setZero();
    desired_ee_vel.setZero();

    return controller_interface::return_type::OK;
  }

// Update from reference joint states
  controller_interface::return_type AdmittanceRule::update(
      const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
      const geometry_msgs::msg::Wrench &measured_wrench,
      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
      const rclcpp::Duration &period,
      trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_state) {

    desired_joint_state = reference_joint_state;
    bool no_failure = true;

    no_failure &= ik_->calculate_link_transform(desired_joint_state.positions, parameters_.end_effector_name_,  desired_ee_transform_vec);
    memcpy(desired_ee_transform.data(),desired_ee_transform_vec.data(), 16*sizeof(double));

    no_failure &= ik_->calculate_link_transform(current_joint_state.positions, parameters_.end_effector_name_, cur_ee_transform_vec);
    memcpy(cur_ee_transform.data(),cur_ee_transform_vec.data(), 16*sizeof(double));

    no_failure &= ik_->calculate_link_transform(current_joint_state.positions, parameters_.sensor_frame_, cur_sensor_transform_vec);
    memcpy(cur_sensor_transform.data(),cur_sensor_transform_vec.data(), 16*sizeof(double));

    no_failure &= ik_->calculate_link_transform(current_joint_state.positions, parameters_.control_frame_, cur_control_transform_vec);
    memcpy(cur_control_transform.data(),cur_control_transform_vec.data(), 16*sizeof(double));

    auto cur_control_transform_inv = invert_transform(cur_control_transform);
    auto cur_control_rot_inv = cur_control_transform_inv(Eigen::seq(0,2), Eigen::seq(0,2));
    auto cur_control_rot = cur_control_transform(Eigen::seq(0,2), Eigen::seq(0,2));
    auto cur_sensor_rot = cur_sensor_transform(Eigen::seq(0,2), Eigen::seq(0,2));

    desired_ee_vel(Eigen::seq(0,2)) = cur_control_rot_inv*desired_ee_vel(Eigen::seq(0,2));
    desired_ee_vel(Eigen::seq(3,5)) = cur_control_rot_inv*desired_ee_vel(Eigen::seq(3,5));

    // apply filter and update wrench vector
    process_wrench_measurements(measured_wrench);
    // transform into control frame
    Eigen::Vector<double,6> wrench_control;
    wrench_control(Eigen::seq(0, 2)) = cur_control_rot_inv * cur_sensor_rot * wrench(Eigen::seq(0, 2));
    wrench_control(Eigen::seq(3, 5)) = cur_control_rot_inv * cur_sensor_rot * wrench(Eigen::seq(3, 5));

    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
    calculate_admittance_rule(wrench_control, period);

    // transform admittance values back to base frame
    // velocity
    Eigen::Vector3d admittance_velocity_cf = cur_control_rot*admittance_velocity_(Eigen::seq(0,2));
    memcpy(admittance_velocity_vec.data(),admittance_velocity_cf.data(), 3*sizeof(double));
    Eigen::Vector3d admittance_angular_vel_cf = cur_control_rot*admittance_velocity_(Eigen::seq(3,5));
    memcpy(admittance_velocity_vec.data()+3, admittance_angular_vel_cf.data(), 3*sizeof(double));
    // velocity acceleration
    Eigen::Vector3d admittance_accel_cf = cur_control_rot * admittance_acceleration_(Eigen::seq(0, 2));
    memcpy(admittance_acceleration_vec.data(),admittance_accel_cf.data(), 3*sizeof(double));
    Eigen::Vector3d admittance_angular_accel_cf = cur_control_rot * admittance_acceleration_(Eigen::seq(3, 5));
    memcpy(admittance_acceleration_vec.data()+3, admittance_angular_vel_cf.data(), 3*sizeof(double));

    // calculate robot joint velocity from admittance velocity
    no_failure &= ik_->convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_velocity_vec, joint_vel);
    // calculate robot joint acceleration from admittance velocity
    no_failure &= ik_->convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_acceleration_vec, joint_acc);

    if (!no_failure){
      desired_joint_state = reference_joint_state;
      return controller_interface::return_type::ERROR;
    }

    // update joint desired joint state
    for (auto i = 0ul; i < reference_joint_state.positions.size(); i++) {
      joint_pos[i] += joint_vel[i] * (1.0 / 1000.0);//- .2 * joint_pos[i] * (1.0 / 1000.0);
      desired_joint_state.positions[i] = reference_joint_state.positions[i] + joint_pos[i];
    }
    for (auto i = 0ul; i < reference_joint_state.velocities.size(); i++) {
      desired_joint_state.velocities[i] = reference_joint_state.velocities[i] + joint_vel[i];
    }
    for (auto i = 0ul; i < reference_joint_state.accelerations.size(); i++) {
      desired_joint_state.accelerations[i] = joint_acc[i];
    }

    return controller_interface::return_type::OK;
  }

  void AdmittanceRule::calculate_admittance_rule(
      const Eigen::Vector<double, 6> &wrench,
      const rclcpp::Duration &period
  ) {
    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
    std::array<double,6> pose_error = {};
    for (size_t axis = 0; axis < 3; ++axis) { //TODO 6
      if (parameters_.selected_axes_[axis]) {
          pose_error[axis] = -admittance_position_(axis, 3);
        admittance_acceleration_[axis] = (1.0 / parameters_.mass_[axis]) * (wrench[axis] +
                                                                            (parameters_.damping_[axis] *
                                                                            (desired_ee_vel[axis] -
                                                                             admittance_velocity_[axis])) +
                                                                            (parameters_.stiffness_[axis] *
                                                                            pose_error[axis]));
        admittance_velocity_[axis] += admittance_acceleration_[axis] * (1.0 / 1000);//period.nanoseconds() TODO fix time isssue
        admittance_position_(axis, 3) += admittance_velocity_[axis] * (1.0 / 1000);
      }
    }

    auto R = admittance_position_(Eigen::seq(0,2), Eigen::seq(0,2));
    Eigen::Vector3d V = get_rotation_axis(R);
    double theta = acos((1.0/2.0)*(R.trace()-1));
    // if trace of the rotation matrix derivative is negative, then rotation axis needs to be flipped
    auto tmp = V[0]*(R(1,2)-R(2,1))+V[1]*(R(2,0)-R(0,2))
        +V[2]*(R(0,1)-R(1,0));
    double sign = (tmp >= 0) ? 1.0 : -1.0;

    for (size_t axis = 3; axis < 6; ++axis) {
      if (parameters_.selected_axes_[axis]) {
        pose_error[axis] = sign*theta*V(axis-3);
        admittance_acceleration_[axis] = (1.0 / parameters_.mass_[axis]) * (wrench[axis] +
                                                                            (parameters_.damping_[axis] *
                                                                            (desired_ee_vel[axis] -
                                                                             admittance_velocity_[axis])) +
                                                                            (parameters_.stiffness_[axis] *
                                                                            pose_error[axis]));
        admittance_velocity_[axis] += admittance_acceleration_[axis] * (1.0 / 1000);//period.nanoseconds() TODO fix time issue
      }
    }

    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -admittance_velocity_[2+3],  admittance_velocity_[1+3],
                      admittance_velocity_[2+3],       0, -admittance_velocity_[0+3],
                      -admittance_velocity_[1+3], admittance_velocity_[0+3],       0;

    Eigen::Matrix3d R_dot = skew_symmetric*R;
    R += R_dot*(1.0 / 1000);
    R = normalize_rotation(R);
    admittance_position_(Eigen::seq(0,2), Eigen::seq(0,2)) = R;
  }

  Eigen::Vector3d AdmittanceRule::get_rotation_axis(const Eigen::Matrix3d& R) const{

    Eigen::Vector3d V;
    bool solved = false;
    double v1 = 1;
    double v2, v3;
    double R_buffer[9];

    // rotate matrix rows and columns to hit every edge case
    for (int i = 0; i < 3; i++){

      for(int r = 0; r < 3; r++){
        for(int c = 0; c < 3; c++){
          R_buffer[((c*3 - 3*i) % 9) + ((r - i) % 3)] = R(r,c);
        }
      }
      auto R11 = R_buffer[0];
      auto R12 = R_buffer[1];
      auto R32 = R_buffer[7];
      auto R13 = R_buffer[2];
      auto R33 = R_buffer[8];
      // degenerate: one axis rotation
      if (abs(R12+R13) < EPSILON && R11 > 0){
        v2 = 0;
        v3 = 0;
        V[i % 3] = v1;
        V[(i+1) % 3] = v2;
        V[(i+2) % 3] = v3;
        solved = true;
        break;
      }
      // degenerate: two axis rotation
      if (abs(R12 + R13) < EPSILON && R11 < 0){
        v1 = 0;
        v2 = 1;
        v3 = R32/(1-R33);
        V[i % 3] = v1;
        V[(i+1) % 3] = v2;
        V[(i+2) % 3] = v3;
        solved = true;
        break;
      }
    }

    // general case: three axis rotation
    if (!solved){
      v3 = (-R(0,1) -((R(1,1)-1)*(1-R(0,0)))/R(1,0));
      // if v3 is zero, special case
      if (abs(v3) > EPSILON) {
        v3 = v3 / (R(2,1) - ((R(1,1) - 1) * (R(2,0))) / (R(1,0)));
      }
      v2 = (1-R(0,0)-R(2,0)*v3)/R(1,0);

      V[0] = v1;
      V[1] = v2;
      V[2] = v3;
    }

    V.normalize();
    return V;
  }

  Eigen::Matrix3d AdmittanceRule::normalize_rotation(Eigen::Matrix3d R){
    // enforce orthonormal constraint

    Eigen::Vector3d R_0 = R(Eigen::seq(0,2), Eigen::seq(0,0));
    R_0.normalize();
    Eigen::Vector3d R_1 = R(Eigen::seq(0,2), Eigen::seq(1,1));
    R_1.normalize();
    Eigen::Vector3d R_2 = R(Eigen::seq(0,2), Eigen::seq(2,2));
    R_2.normalize();

    double drift = R_0.dot(R_1);
    R_1 += R_0*(-drift);
    R_1.normalize();

    R_2 = R_0.cross(R_1);
    R_2.normalize();

    Eigen::Matrix3d R_out;
    R_out.col(0) << R_0;
    R_out.col(1) << R_1;
    R_out.col(2) << R_2;

    return R_out;
  }

  Eigen::Matrix4d AdmittanceRule::invert_transform(const Eigen::Matrix4d & T){
    Eigen::Matrix4d T_inv = T;
    Eigen::Matrix3d R = T_inv(Eigen::seq(0,2), Eigen::seq(0,2));
    T_inv(Eigen::seq(0,2), Eigen::seq(0,2)) = R.transpose();
    T_inv(Eigen::seq(0,2), Eigen::seq(3,3)) = -R.transpose()*T_inv(Eigen::seq(0,2), Eigen::seq(3,3));
    return T_inv;

  }

  controller_interface::return_type AdmittanceRule::get_controller_state(
      control_msgs::msg::AdmittanceControllerState &state_message) {
    // TODO update this
    //   state_message.input_wrench_control_frame = reference_wrench_control_frame_;
//    state_message.input_pose_control_frame = reference_pose_ik_base_frame_;
//    state_message.measured_wrench = measured_wrench_;
//    state_message.measured_wrench_filtered = measured_wrench_filtered_;
//    state_message.measured_wrench_control_frame = measured_wrench_ik_base_frame_;
//
//    state_message.admittance_rule_calculated_values = admittance_rule_calculated_values_;
//
//    state_message.current_pose = current_pose_ik_base_frame_;
//    state_message.desired_pose = admittance_pose_ik_base_frame_;
//    // TODO(destogl): Enable this field for debugging.
////   state_message.relative_admittance = sum_of_admittance_displacements_;
//    state_message.relative_desired_pose = relative_admittance_pose_ik_base_frame_;

    return controller_interface::return_type::OK;
  }

  void AdmittanceRule::process_wrench_measurements(
      const geometry_msgs::msg::Wrench &measured_wrench
  ) {
    wrench[0] = filters::exponentialSmoothing(
                                              measured_wrench.force.x, wrench[0], alpha);
    wrench[1] = filters::exponentialSmoothing(
                                              measured_wrench.force.y, wrench[1], alpha);
    wrench[2] = filters::exponentialSmoothing(
                                              measured_wrench.force.z, wrench[2], alpha);

    wrench[3] = filters::exponentialSmoothing(
                                              measured_wrench.torque.x, wrench[3], alpha);
    wrench[4]= filters::exponentialSmoothing(
                                             measured_wrench.torque.y, wrench[4], alpha);
    wrench[5] = filters::exponentialSmoothing(
                                              measured_wrench.torque.z, wrench[5], alpha);
  }

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
