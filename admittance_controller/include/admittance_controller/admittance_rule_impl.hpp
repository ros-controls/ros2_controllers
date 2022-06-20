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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_

#include "admittance_controller/admittance_rule.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace admittance_controller {

  controller_interface::return_type
  AdmittanceRule::configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int num_joints) {
    // configure admittance rule using num_joints and load kinematics interface

    num_joints_ = num_joints;

    // initialize memory and values to zero  (non-realtime function)
    reset();

    // set parameters values
    memcpy(cog_.data(), parameters_.cog_.data(), 3 * sizeof(double));
    ee_weight.setZero();
    ee_weight[2] = -parameters_.force_;

    // Load the differential IK plugin
    if (!parameters_.kinematics_plugin_name_.empty()) {
      try {
        // TODO(destogl): add "kinematics_interface" into separate package and then rename the package in
        // the next line from "admittance_controller" to "kinematics_base_plugin"
        kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>>(
            "kdl_plugin", "kinematics_interface::KinematicsBaseClass");
        kinematics_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
            kinematics_loader_->createUnmanagedInstance(parameters_.kinematics_plugin_name_));
        if (!kinematics_->initialize(node, parameters_.end_effector_name_)) {
          return controller_interface::return_type::ERROR;
        }
      }
      catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
                     parameters_.kinematics_plugin_name_.c_str(), ex.what());
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

    //allocate dynamic buffers
    joint_buffer_vec.assign(num_joints_, 0.0);
    transform_buffer_vec.assign(16, 0.0);
    cart_buffer_vec.assign(6, 0.0);

    // admittance state vectors in joint space
    joint_pos.resize(num_joints_, 0.0);
    joint_vel.resize(num_joints_, 0.0);
    joint_acc.resize(num_joints_, 0.0);

    // transforms
    cur_ee_transform.setIdentity();
    reference_ee_transform.setIdentity();
    cur_sensor_transform.setIdentity();
    cur_control_transform.setIdentity();
    cog_transform.setIdentity();

    // admittance values
    admittance_position_.setIdentity();
    admittance_velocity_.setZero();
    admittance_acceleration_.setZero();
    wrench.setZero();

    return controller_interface::return_type::OK;
  }

// Update from reference joint states
  controller_interface::return_type AdmittanceRule::update(
      const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
      const geometry_msgs::msg::Wrench &measured_wrench,
      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
      const rclcpp::Duration &period,
      trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_state) {

    // keep track of failed kinematics interface calls
    bool success = true;
    double dt = period.seconds()+((double) period.nanoseconds())*1E-9;

    // get all needed transforms
    reference_ee_transform = get_transform(reference_joint_state.positions, parameters_.end_effector_name_, success);
    cur_ee_transform = get_transform(current_joint_state.positions, parameters_.end_effector_name_, success);
    cur_sensor_transform = get_transform(current_joint_state.positions, parameters_.sensor_frame_, success);
    cur_control_transform = get_transform(current_joint_state.positions, parameters_.control_frame_, success);
    cog_transform = get_transform(current_joint_state.positions, parameters_.cog_frame_, success);
    auto cur_control_transform_inv = invert_transform(cur_control_transform);

    // get all needed rotations
    auto cur_control_rot = cur_control_transform.block<3, 3>(0, 0);
    auto cur_control_rot_inv = cur_control_transform_inv.block<3, 3>(0, 0);
    auto cur_sensor_rot = cur_sensor_transform.block<3, 3>(0, 0);
    auto cog_rot = cog_transform.block<3, 3>(0, 0);

    // apply filter and update wrench vector
    wrench = process_wrench_measurements(measured_wrench, wrench);

    // transform wrench into base frame
    Eigen::Matrix<double, 6, 1> wrench_base;
    wrench_base.block<3, 1>(0, 0) = cur_sensor_rot * wrench.block<3, 1>(0, 0);
    wrench_base.block<3, 1>(3, 0) = cur_sensor_rot * wrench.block<3, 1>(3, 0);

    // apply gravity compensation
    wrench_base[2] -= ee_weight[2];
    wrench_base.block<3, 1>(3, 0) -= (cog_rot * cog_).cross(ee_weight);

    // transform wrench into control frame
    Eigen::Matrix<double, 6, 1> wrench_control;
    wrench_control.block<3, 1>(0, 0) = cur_control_rot_inv * wrench_base.block<3, 1>(0, 0);
    wrench_control.block<3, 1>(3, 0) = cur_control_rot_inv * wrench_base.block<3, 1>(3, 0);

    // calculate desired velocity in control frame
    auto desired_vel = cur_control_rot_inv * admittance_velocity_.reshaped(3, 2) * use_feedforward_commanded_input_*0; // TODO fix this

    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
    calculate_admittance_rule(wrench_control, desired_vel.reshaped(6, 1), dt);

    // transform admittance values back to base frame
    auto admittance_velocity_base = cur_control_rot * admittance_velocity_.reshaped(3, 2);
    convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_velocity_base.reshaped(6, 1),
                                             joint_vel, success);
    auto admittance_acceleration_base = cur_control_rot * admittance_acceleration_.reshaped(3, 2);
    convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_acceleration_base.reshaped(6, 1),
                                             joint_acc, success);

    // if a failure occurred during any kinematics interface calls, return an error and don't modify the desired reference
    if (!success) {
      desired_joint_state = reference_joint_state;
      return controller_interface::return_type::ERROR;
    }

    // update joint desired joint state
    for (auto i = 0ul; i < reference_joint_state.positions.size(); i++) {
      joint_pos[i] += joint_vel[i] * dt;//- .2 * joint_pos[i] * (1.0 / 1000.0);
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
      const Eigen::Matrix<double, 6, 1> &wrench,
      const Eigen::Matrix<double, 6, 1> &desired_vel,
      const double dt
  ) {
    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)

    for (size_t axis = 0; axis < 3; ++axis) {
      if (parameters_.selected_axes_[axis]) {
        double pose_error = -admittance_position_(axis, 3);
        admittance_acceleration_[axis] = (1.0 / parameters_.mass_[axis]) * (wrench[axis] +
                                                                            (parameters_.damping_[axis] *
                                                                             (desired_vel[axis] -
                                                                              admittance_velocity_[axis])) +
                                                                            (parameters_.stiffness_[axis] *
                                                                             pose_error));
        admittance_velocity_[axis] +=
            admittance_acceleration_[axis] * dt;
        admittance_position_(axis, 3) += admittance_velocity_[axis] * dt;
      }
    }

//    auto R = admittance_position_(Eigen::seq(0,2), Eigen::seq(0,2));
    Eigen::Matrix<double, 3 ,3> R = admittance_position_.block<3, 3>(0, 0);
    Eigen::Vector3d V = get_rotation_axis(R);
    double theta = acos((1.0 / 2.0) * (R.trace() - 1));
    // if trace of the rotation matrix derivative is negative, then rotation axis needs to be flipped
    auto tmp = V[0] * (R(1, 2) - R(2, 1)) + V[1] * (R(2, 0) - R(0, 2))
               + V[2] * (R(0, 1) - R(1, 0));
    double sign = (tmp >= 0) ? 1.0 : -1.0;

    for (size_t axis = 3; axis < 6; ++axis) {
      if (parameters_.selected_axes_[axis]) {
        double pose_error = sign * theta * V(axis - 3);
        admittance_acceleration_[axis] = (1.0 / parameters_.mass_[axis]) * (wrench[axis] +
                                                                            (parameters_.damping_[axis] *
                                                                             (desired_vel[axis] -
                                                                              admittance_velocity_[axis])) +
                                                                            (parameters_.stiffness_[axis] *
                                                                             pose_error));
        admittance_velocity_[axis] +=
            admittance_acceleration_[axis] * dt;
      }
    }

    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -admittance_velocity_[2 + 3], admittance_velocity_[1 + 3],
        admittance_velocity_[2 + 3], 0, -admittance_velocity_[0 + 3],
        -admittance_velocity_[1 + 3], admittance_velocity_[0 + 3], 0;

    Eigen::Matrix3d R_dot = skew_symmetric * R;
    R += R_dot * dt;
    normalize_rotation(R);
    admittance_position_.block<3, 3>(0, 0) = R;
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
      if (abs(R12 + R13) < ROT_AXIS_EPSILON && R11 > 0) {
        v2 = 0;
        v3 = 0;
        V[i % 3] = v1;
        V[(i + 1) % 3] = v2;
        V[(i + 2) % 3] = v3;
        solved = true;
        break;
      }
      // degenerate: two axis rotation
      if (abs(R12 + R13) < ROT_AXIS_EPSILON && R11 < 0) {
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
      if (abs(v3) > ROT_AXIS_EPSILON) {
        v3 = v3 / (R(2, 1) - ((R(1, 1) - 1) * (R(2, 0))) / (R(1, 0)));
      }
      v2 = (1 - R(0, 0) - R(2, 0) * v3) / R(1, 0);

      V[0] = v1;
      V[1] = v2;
      V[2] = v3;
    }

    V.normalize();
    return V;
  }

  void AdmittanceRule::normalize_rotation(Eigen::Matrix3d& R) {
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

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
  AdmittanceRule::get_transform(const std::vector<double> &positions, const std::string &link_name, bool &success) {
    success &= kinematics_->calculate_link_transform(positions, link_name, transform_buffer_vec);
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transform;
    memcpy(transform.data(), transform_buffer_vec.data(), 16 * sizeof(double));
    return transform;

  }

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
  AdmittanceRule::invert_transform(Eigen::Matrix<double, 4, 4, Eigen::ColMajor> &T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix4d T_inv = T;
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * T_inv.block<3, 1>(0, 3);
    return T_inv;
  }

  void AdmittanceRule::convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &positions,
                                                                const Eigen::Matrix<double, 6, 1> &cartesian_delta,
                                                                std::vector<double> &joint_delta, bool &success) {
    memcpy(cart_buffer_vec.data(), cartesian_delta.data(), 6 * sizeof(double));
    success &= kinematics_->convert_cartesian_deltas_to_joint_deltas(positions, cart_buffer_vec, joint_delta);
  }

  controller_interface::return_type AdmittanceRule::get_controller_state(
      control_msgs::msg::AdmittanceControllerState &state_message) {
    // TODO update this
//   state_message.input_wrench_control_frame = reference_wrench_control_frame_;
//    state_message.input_pose_control_frame = reference_pose_kinematics_base_frame_;
//    state_message.measured_wrench = measured_wrench_;
//    state_message.measured_wrench_filtered = measured_wrench_filtered_;
//    state_message.measured_wrench_control_frame = measured_wrench_kinematics_base_frame_;
//
//    state_message.admittance_rule_calculated_values = admittance_rule_calculated_values_;
//
//    state_message.current_pose = current_pose_kinematics_base_frame_;
//    state_message.desired_pose = admittance_pose_kinematics_base_frame_;
//    // TODO(destogl): Enable this field for debugging.
////   state_message.relative_admittance = sum_of_admittance_displacements_;
//    state_message.relative_desired_pose = relative_admittance_pose_kinematics_base_frame_;

    return controller_interface::return_type::OK;
  }

  Eigen::Matrix<double, 6, 1> AdmittanceRule::process_wrench_measurements(
      const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 6, 1> &last_wrench
  ) {
    Eigen::Matrix<double, 6, 1> new_wrench;

    new_wrench[0] = filters::exponentialSmoothing(
        measured_wrench.force.x, last_wrench[0], alpha);
    new_wrench[1] = filters::exponentialSmoothing(
        measured_wrench.force.y, last_wrench[1], alpha);
    new_wrench[2] = filters::exponentialSmoothing(
        measured_wrench.force.z, last_wrench[2], alpha);
    new_wrench[3] = filters::exponentialSmoothing(
        measured_wrench.torque.x, last_wrench[3], alpha);
    new_wrench[4] = filters::exponentialSmoothing(
        measured_wrench.torque.y, last_wrench[4], alpha);
    new_wrench[5] = filters::exponentialSmoothing(
        measured_wrench.torque.z, last_wrench[5], alpha);
    return new_wrench;

  }

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
