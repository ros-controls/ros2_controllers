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

#include <tf2_ros/transform_listener.h>
#include "admittance_controller/admittance_rule.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"

namespace admittance_controller {

  controller_interface::return_type
  AdmittanceRule::configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int num_joints) {
    // configure admittance rule using num_joints and load kinematics interface

    num_joints_ = num_joints;

    // initialize ROS functions
    clock_ = node->get_clock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize memory and values to zero  (non-realtime function)
    reset();

    // Load the differential IK plugin
    if (!parameters_->kinematics_.plugin_name_.empty()) {
      try {
        // TODO(destogl): add "kinematics_interface" into separate package and then rename the package in
        // the next line from "admittance_controller" to "kinematics_base_plugin"
        kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>>(
            "kdl_plugin", "kinematics_interface::KinematicsBaseClass");
        kinematics_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
            kinematics_loader_->createUnmanagedInstance(parameters_->kinematics_.plugin_name_));
        if (!kinematics_->initialize(node, parameters_->kinematics_.tip_)) {
          return controller_interface::return_type::ERROR;
        }
      }
      catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
                     parameters_->kinematics_.plugin_name_.c_str(), ex.what());
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

    damping_.assign(6, 0.0);
    mass_.assign(6, 0.0);
    stiffness_.assign(6, 0.0);
    selected_axes_.assign(6, 0.0);

    // admittance state vectors in joint space
    joint_pos.resize(num_joints_, 0.0);
    joint_vel.resize(num_joints_, 0.0);
    joint_acc.resize(num_joints_, 0.0);

    // transforms
    ee_transform.setIdentity();
    reference_ee_transform.setIdentity();
    sensor_transform.setIdentity();
    control_transform.setIdentity();
    cog_transform.setIdentity();
    base_link_transform_.setIdentity();

    // admittance values
    admittance_position_.setIdentity();
    admittance_velocity_.setZero();
    admittance_acceleration_.setZero();
    wrench_.setZero();

    return controller_interface::return_type::OK;
  }

// Update from reference joint states
  controller_interface::return_type AdmittanceRule::update(
      const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
      const geometry_msgs::msg::Wrench &measured_wrench,
      const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
      const rclcpp::Duration &period,
      trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_state) {

    // update param values
    memcpy(cog_.data(), parameters_->gravity_compensation_.CoG_.pos_.data(), 3*sizeof(double ));
    ee_weight.setZero();
    ee_weight[2] = -parameters_->gravity_compensation_.CoG_.force_;

    mass_ = parameters_->admittance_.mass_;
    selected_axes_= parameters_->admittance_.selected_axes_;
    stiffness_ = parameters_->admittance_.stiffness_;
    for (auto i = 0ul; i < damping_.size(); ++i)
    {
      damping_[i] = parameters_->admittance_.damping_ratio_[i] * 2 * sqrt(mass_[i] * stiffness_[i]);
    }

    // keep track of failed kinematics interface calls
    bool success = true;
    double dt = period.seconds() + ((double) period.nanoseconds()) * 1E-9;

    // get all needed transforms
    // since the base_link_transform transform in the fixed_world_frame_ cannot be determined from the URDF,
    // we must resort to TF

    // TODO check if world frame is external to robot
//    if (parameters_.fixed_world_frame_ != parameters_.kinematics_base_frame_){
//    base_link_transform_ = find_transform(parameters_->fixed_world_frame_.id_,
//                                          parameters_->kinematics_.base_, success);
    base_link_transform_ = get_transform(current_joint_state.positions, parameters_->kinematics_.base_, success);
//    }
//
//    control_transform = find_transform(parameters_->fixed_world_frame_.id_,
//                                       parameters_->control_.frame_.id_, success);
    control_transform = get_transform(current_joint_state.positions, parameters_->control_.frame_.id_, success);

    reference_ee_transform = get_transform(reference_joint_state.positions, parameters_->kinematics_.tip_, success);
    ee_transform = get_transform(current_joint_state.positions, parameters_->kinematics_.tip_, success);
    sensor_transform = get_transform(current_joint_state.positions, parameters_->ft_sensor_.frame_.id_, success);
    cog_transform = get_transform(current_joint_state.positions, parameters_->gravity_compensation_.frame_.id_, success);

    // get all needed rotations
    auto cur_control_rot = control_transform.block<3, 3>(0, 0);
    auto cur_control_rot_inv = cur_control_rot.transpose();
    auto cur_sensor_rot = sensor_transform.block<3, 3>(0, 0);
    auto cog_rot = cog_transform.block<3, 3>(0, 0);
    auto base_link_rot = base_link_transform_.block<3, 3>(0, 0);

    // apply filter and update wrench_ vector
    process_wrench_measurements(measured_wrench, base_link_rot * cur_sensor_rot, base_link_rot * cog_rot);

    // transform wrench_ into control frame
    Eigen::Matrix<double, 3, 2> wrench_control = cur_control_rot_inv * wrench_;

    // calculate desired cartesian velocity in control frame
    Eigen::Matrix<double, 3, 2> desired_vel = convert_joint_deltas_to_cartesian_deltas(current_joint_state.positions,
                                                                                       current_joint_state.velocities,
                                                                                       success);

    desired_vel = cur_control_rot_inv * desired_vel * parameters_->use_feedforward_commanded_input_;

    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
    calculate_admittance_rule(wrench_control, desired_vel, dt);

    // transform admittance values back to base frame
    auto admittance_velocity_base = cur_control_rot * admittance_velocity_;
    convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_velocity_base,
                                             joint_vel, success);
    auto admittance_acceleration_base = cur_control_rot * admittance_acceleration_;
    convert_cartesian_deltas_to_joint_deltas(current_joint_state.positions, admittance_acceleration_base,
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

    // update controller message state
    eigen_to_msg(wrench_, "base_link", state_message_.input_wrench_command);
//    state_message_.input_wrench_control_frame.wrench = wrench_control;

//    geometry_msgs/WrenchStamped input_wrench_command  #commanded input wrench_ for the controller
//    geometry_msgs/PoseStamped input_pose_command     #commanded input pose for the controller
//    trajectory_msgs/JointTrajectory input_joint_command  # commanded input JointTrajectory (used only first point)
//
//    geometry_msgs/WrenchStamped input_wrench_control_frame  # input wrench_ transformed into control frame
//    geometry_msgs/PoseStamped input_pose_control_frame     # input pose transformed into control frame
//
//    geometry_msgs/WrenchStamped measured_wrench           # measured wrench_ from the sensor (sensor frame)
//    geometry_msgs/WrenchStamped measured_wrench_filtered  # measured wrench_ after low-pass and gravity compensation filters in sensor frame
//    geometry_msgs/WrenchStamped measured_wrench_control_frame  # transformed measured wrench_ to control frame
//    geometry_msgs/WrenchStamped measured_wrench_endeffector_frame  # measured wrench_ transformed to endeffector
//
//    trajectory_msgs/JointTrajectoryPoint admittance_rule_calculated_values  # Values calculated by admittance rule (Cartesian space: [x, y, z, rx, ry, rz]) - position = pose_error; effort = measured_wrench
//
//    geometry_msgs/PoseStamped current_pose                # Current pose from FK
//    geometry_msgs/PoseStamped desired_pose                # goal pose to send to the robot
//    geometry_msgs/TransformStamped relative_desired_pose  # relative pose from the actual pose as result of the admittance rule
//
//    string[] joint_names
//    trajectory_msgs/JointTrajectoryPoint desired_joint_state  # result of IK from goal_pose_command
//    trajectory_msgs/JointTrajectoryPoint actual_joint_state   # read from the hardware
//    trajectory_msgs/JointTrajectoryPoint error_joint_state



    return controller_interface::return_type::OK;
  }

  void AdmittanceRule::calculate_admittance_rule(
      const Eigen::Matrix<double, 3, 2> &wrench,
      const Eigen::Matrix<double, 3, 2> &desired_vel,
      const double dt
  ) {
    // Compute admittance control law: F = M*a + D*v + S*(x - x_d)

    for (size_t axis = 0; axis < 3; ++axis) {
      if (selected_axes_[axis]) {
        double pose_error = -admittance_position_(axis, 3);
        admittance_acceleration_(axis, 0) = (1.0 / mass_[axis]) * (wrench(axis, 0) +
                                                                  (damping_[axis] *
                                                                   (desired_vel(axis, 0) -
                                                                    admittance_velocity_(axis, 0))) +
                                                                  (stiffness_[axis] *
                                                                   pose_error));
        admittance_velocity_(axis, 0) +=
            admittance_acceleration_(axis, 0) * dt;
        admittance_position_(axis, 3) += admittance_velocity_(axis, 0) * dt;
      }
    }

//    auto R = admittance_position_(Eigen::seq(0,2), Eigen::seq(0,2));
    Eigen::Matrix<double, 3, 3> R = admittance_position_.block<3, 3>(0, 0);
    Eigen::Vector3d V = get_rotation_axis(R);
    double theta = acos((1.0 / 2.0) * (R.trace() - 1));
    // if trace of the rotation matrix derivative is negative, then rotation axis needs to be flipped
    auto tmp = V[0] * (R(1, 2) - R(2, 1)) + V[1] * (R(2, 0) - R(0, 2))
               + V[2] * (R(0, 1) - R(1, 0));
    double sign = (tmp >= 0) ? 1.0 : -1.0;

    for (size_t axis = 0; axis < 3; ++axis) {
      if (selected_axes_[axis + 3]) {
        double pose_error = sign * theta * V(axis);
        admittance_acceleration_(axis, 1) = (1.0 / mass_[axis + 3]) * (wrench(axis, 1) +
                                                                      (damping_[axis + 3] *
                                                                       (desired_vel(axis, 1) -
                                                                        admittance_velocity_(axis, 1))) +
                                                                      (stiffness_[axis + 3] *
                                                                       pose_error));
        admittance_velocity_(axis, 1) +=
            admittance_acceleration_(axis, 1) * dt;
      }
    }

    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -admittance_velocity_(2, 1), admittance_velocity_(1, 1),
        admittance_velocity_(2, 1), 0, -admittance_velocity_(0, 1),
        -admittance_velocity_(1, 1), admittance_velocity_(0, 1), 0;

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

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
  AdmittanceRule::get_transform(const std::vector<double> &positions, const std::string &link_name, bool &success) {
    success &= kinematics_->calculate_link_transform(positions, link_name, transform_buffer_vec);
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transform;
    memcpy(transform.data(), transform_buffer_vec.data(), 16 * sizeof(double));
    return transform;

  }

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
  AdmittanceRule::find_transform(const std::string &target_frame, const std::string &source_frame, bool &success) {
    //Parameters
    // target_frame: The frame to which data should be transformed
    //source_frame: The frame where the data originated

    Eigen::Matrix<double, 4, 4> transform;
    transform.setIdentity();
    try {
      auto transform_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      transform.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>(tf2::transformToKDL(transform_msg).M.data);
      transform(0, 3) = transform_msg.transform.translation.x;
      transform(1, 3) = transform_msg.transform.translation.y;
      transform(2, 3) = transform_msg.transform.translation.z;
    } catch (const tf2::TransformException &e) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(
          rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
      success = false;
    }
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
                                                                const Eigen::Matrix<double, 3, 2> &cartesian_delta,
                                                                std::vector<double> &joint_delta, bool &success) {
    memcpy(cart_buffer_vec.data(), cartesian_delta.data(), 6 * sizeof(double));
    success &= kinematics_->convert_cartesian_deltas_to_joint_deltas(positions, cart_buffer_vec, joint_delta);
  }

  Eigen::Matrix<double, 3, 2>
  AdmittanceRule::convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &positions,
                                                           const std::vector<double> &joint_delta,
                                                           bool &success) {
    Eigen::Matrix<double, 3, 2> cartesian_delta;
    success &= kinematics_->convert_joint_deltas_to_cartesian_deltas(positions, joint_delta, cart_buffer_vec);
    memcpy(cartesian_delta.data(), cart_buffer_vec.data(), 6 * sizeof(double));
    return cartesian_delta;
  }

  control_msgs::msg::AdmittanceControllerState
  AdmittanceRule::get_controller_state(control_msgs::msg::AdmittanceControllerState &state_message) {
//    // TODO update this
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

    return state_message_;//controller_interface::return_type::OK;
  }

  void AdmittanceRule::process_wrench_measurements(
      const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 3, 3> &cur_sensor_rot,
      const Eigen::Matrix<double, 3, 3> &cog_rot
  ) {
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> new_wrench;
    new_wrench(0, 0) = measured_wrench.force.x;
    new_wrench(1, 0) = measured_wrench.force.y;
    new_wrench(2, 0) = measured_wrench.force.z;
    new_wrench(0, 1) = measured_wrench.torque.x;
    new_wrench(1, 1) = measured_wrench.torque.y;
    new_wrench(2, 1) = measured_wrench.torque.z;

    // transform to base frame
    Eigen::Matrix<double, 3, 2> new_wrench_base = cur_sensor_rot * new_wrench;

    // apply gravity compensation
    new_wrench_base(2, 0) -= ee_weight[2];
    new_wrench_base.block<3, 1>(0, 1) -= (cog_rot * cog_).cross(ee_weight);

    for (auto i = 0; i < 6; i++) {
      wrench_(i) = filters::exponentialSmoothing(
          new_wrench_base(i), wrench_(i), parameters_->ft_sensor_.filter_coefficient_);
    }

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

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
