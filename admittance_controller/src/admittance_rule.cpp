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
/// \author: Denis Stogl

#include "admittance_controller/admittance_rule.hpp"

#include "angles/angles.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace {  // Utility namespace

void get_rpy_difference_between_two_quaternions(const geometry_msgs::msg::Quaternion quat1,
                                                const geometry_msgs::msg::Quaternion quat2, std::array<double, 3> vector_out)
{
  tf2::Quaternion q1(quat1.x, quat1.y, quat1.z, quat1.w);
  tf2::Quaternion q2(quat2.x, quat2.y, quat2.z, quat2.w);

  // q1 - q2
  const tf2::Quaternion q_diff = q1 * q2.inverse();
  tf2::Matrix3x3(q_diff).getRPY(vector_out[0], vector_out[1], vector_out[2]);
}

void convert_message_to_array(const geometry_msgs::msg::Pose & msg, std::array<double, 6> & vector_out)
{
  vector_out[0] = msg.position.x;
  vector_out[1] = msg.position.y;
  vector_out[2] = msg.position.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

void convert_message_to_array(
  const geometry_msgs::msg::PoseStamped & msg, std::array<double, 6> & vector_out)
{
  convert_message_to_array(msg.pose, vector_out);
}

void convert_message_to_array(
  const geometry_msgs::msg::Wrench & msg, std::array<double, 6> & vector_out)
{
  vector_out[0] = msg.force.x;
  vector_out[1] = msg.force.y;
  vector_out[2] = msg.force.z;
  vector_out[3] = msg.torque.x;
  vector_out[4] = msg.torque.y;
  vector_out[5] = msg.torque.z;
}

void convert_message_to_array(
  const geometry_msgs::msg::WrenchStamped & msg, std::array<double, 6> & vector_out)
{
  convert_message_to_array(msg.wrench, vector_out);
}

void convert_array_to_message(const std::array<double, 6> & vector, geometry_msgs::msg::Pose & msg_out)
{
  msg_out.position.x = vector[0];
  msg_out.position.y = vector[1];
  msg_out.position.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.orientation.x = q.x();
  msg_out.orientation.y = q.y();
  msg_out.orientation.z = q.z();
  msg_out.orientation.w = q.w();
}

void convert_array_to_message(const std::array<double, 6> & vector, geometry_msgs::msg::PoseStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.pose);
}

void convert_array_to_message(const std::array<double, 6> & vector, geometry_msgs::msg::Wrench & msg_out)
{
  msg_out.force.x = vector[0];
  msg_out.force.y = vector[1];
  msg_out.force.z = vector[2];
  msg_out.torque.x = vector[3];
  msg_out.torque.y = vector[4];
  msg_out.torque.z = vector[5];

}

void convert_array_to_message(const std::array<double, 6> & vector, geometry_msgs::msg::WrenchStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.wrench);
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Transform & msg_out)
{
  msg_out.translation.x = vector[0];
  msg_out.translation.y = vector[1];
  msg_out.translation.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.rotation.x = q.x();
  msg_out.rotation.y = q.y();
  msg_out.rotation.z = q.z();
  msg_out.rotation.w = q.w();
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::TransformStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.transform);
}

}  // utility namespace

namespace admittance_controller
{

controller_interface::return_type AdmittanceRule::configure(rclcpp::Node::SharedPtr node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize variables used in the update loop
  measured_force_.header.frame_id = sensor_frame_;

  current_pose_.header.frame_id = endeffector_frame_;

  relative_desired_pose_.header.frame_id = control_frame_;

  relative_desired_joint_state_vec_.reserve(6);

  // Initialize IK
  ik_ = std::make_shared<IncrementalKinematics>(node, ik_group_name_);

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset()
{
  measured_force_control_frame_arr_.fill(0.0);
  target_pose_control_frame_arr_.fill(0.0);

  current_pose_control_frame_arr_.fill(0.0);

  angles_error_.fill(0.0);

  desired_velocity_arr_.fill(0.0);
  desired_velocity_previous_arr_.fill(0.0);
  desired_acceleration_previous_arr_.fill(0.0);

  // Initialize ik_tip and tool_frame transformations - those are fixed transformations
  tf2::Stamped<tf2::Transform> tf2_transform;
  try {
    auto transform = tf_buffer_->lookupTransform(ik_tip_frame_, endeffector_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform, tf2_transform);
    ik_tip_to_endeffector_frame_tf_ = tf2_transform;
    endeffector_frame_to_ik_tip_tf_ = tf2_transform.inverse();
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_tip_frame_ + "' and '" + endeffector_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

// Update with target Cartesian pose - the main update method!
controller_interface::return_type AdmittanceRule::update(
  const std::array<double, 6> & current_joint_states,
  const geometry_msgs::msg::Wrench & measured_force,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const rclcpp::Duration & period,
  std::array<double, 6> desired_joint_states
)
{
  // Convert inputs to control frame
  transform_message_to_control_frame(target_pose, target_pose_control_frame_);

  // get current states, and transform those into controller frame
  measured_force_.wrench = measured_force;
  // TODO(destogl): Add gravity compensation of measured forces
  measured_force_filtered_ = measured_force_;
  transform_message_to_control_frame(measured_force_, measured_force_control_frame_);

  get_current_pose_of_endeffector_frame(current_pose_);
  transform_message_to_control_frame(current_pose_, current_pose_control_frame_);

  // Convert all data to arrays for simpler calculation
  convert_message_to_array(measured_force_control_frame_, measured_force_control_frame_arr_);
  convert_message_to_array(target_pose_control_frame_, target_pose_control_frame_arr_);
  convert_message_to_array(current_pose_control_frame_, current_pose_control_frame_arr_);

  // Use angle difference calculated with quaternions to avoid confusion with angles
  get_rpy_difference_between_two_quaternions(target_pose_control_frame_.pose.orientation,
                                             current_pose_control_frame_.pose.orientation, angles_error_);

  // Compute admittance control law: F = M*a + D*v + K*(x_d - x)
  for (auto i = 0u; i < 6; ++i) {
    if (selected_axes_[i]) {
      double pose_error = target_pose_control_frame_arr_[i] - measured_force_control_frame_arr_[i];
      if (i >= 3) {
        pose_error = angles_error_[i-3];
      }

      // TODO(destogl): check if velocity is measured from hardware
      const double acceleration = 1 / mass_[i] *
      (measured_force_control_frame_arr_[i] -  damping_[i] * desired_velocity_arr_[i] -
      stiffness_[i] * pose_error);

      desired_velocity_arr_[i] += (desired_acceleration_previous_arr_[i] + acceleration) * 0.5 * period.seconds();

      relative_desired_pose_arr_[i] = (desired_velocity_previous_arr_[i] + desired_velocity_arr_[i]) * 0.5 * period.seconds();

      desired_acceleration_previous_arr_[i] = acceleration;
      desired_velocity_previous_arr_[i] = desired_velocity_arr_[i];
    }
  }

  // TODO: Add here desired vector at the IK tip and not tool!!!!

  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_desired_pose_arr_, relative_desired_pose_);
  tf2::doTransform(current_pose_control_frame_, desired_pose_, relative_desired_pose_);
  ik_tip_to_endeffector_frame(desired_pose_.pose, desired_pose_.pose);

  // Use Jacobian-based IK
  // Calculate desired Cartesian displacement of the robot
  // TODO: replace this with FK in the long term
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(ik_base_frame_, ik_tip_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_base_frame_ + "' and '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  std::vector<double> relative_desired_pose_vec(relative_desired_pose_arr_.begin(), relative_desired_pose_arr_.end());
  if (ik_->convertCartesianDeltasToJointDeltas(
    relative_desired_pose_vec, transform, relative_desired_joint_state_vec_)){
      for (auto i = 0u; i < desired_joint_states.size(); ++i) {
        desired_joint_states[i] = current_joint_states[i] + relative_desired_joint_state_vec_[i];
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the robot.");
      desired_joint_states = current_joint_states;
      return controller_interface::return_type::ERROR;
    }

  // TODO(anyone: enable this when enabling use of IK directly
  // transform = tf_buffer_->lookupTransform(endeffector_frame_, ik_base_frame_, tf2::TimePointZero);
  // tf2::doTransform(desired_pose_, ik_input_pose_, transform);
  // ik_input_pose_.pose = endeffector_to_ik_tip(ik_input_pose_);
  // std::vector<double> ik_sol = ik_solver_->getPositionIK (  ); ...

  return controller_interface::return_type::OK;
}

// Update from target joint deltas
controller_interface::return_type AdmittanceRule::update(
  const std::array<double, 6> & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_force,
  const std::array<double, 6> & target_joint_deltas,
  const rclcpp::Duration & period,
  std::array<double, 6> desired_joint_states)
{
  std::vector<double> target_joint_deltas_vec(target_joint_deltas.begin(), target_joint_deltas.end());
  std::vector<double> target_ik_tip_deltas_vec(6);
  // TODO: replace this with FK in the long term
  geometry_msgs::msg::TransformStamped transform_ik_base_tip;
  try {
    transform_ik_base_tip = tf_buffer_->lookupTransform(ik_base_frame_, ik_tip_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_base_frame_ + "' and '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  if (ik_->convertJointDeltasToCartesianDeltas(target_joint_deltas_vec, transform_ik_base_tip, target_ik_tip_deltas_vec)) {
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of joint deltas to Cartesian deltas failed. Sending current joint values to the robot.");
    desired_joint_states = current_joint_state;
    return controller_interface::return_type::ERROR;
  }

  // TODO(destogl): Use as class variables to avoid memory allocation
  geometry_msgs::msg::PoseStamped current_ik_tip_pose;
  geometry_msgs::msg::TransformStamped current_to_target_ik_pose;
  geometry_msgs::msg::PoseStamped target_ik_tip_pose;
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = endeffector_frame_;
  static geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = endeffector_frame_;
  origin.pose.orientation.w = 1;

  // If FK this is not needed
  // TODO(anyone): Can I just use values from transformation instead calling doTransform?
  tf2::doTransform(origin, current_ik_tip_pose, transform_ik_base_tip);
  convert_array_to_message(target_ik_tip_deltas_vec, current_to_target_ik_pose);
  tf2::doTransform(current_ik_tip_pose, target_ik_tip_pose, current_to_target_ik_pose);

  ik_tip_to_endeffector_frame(target_ik_tip_pose.pose, target_pose.pose);

  return update(current_joint_state, measured_force, target_pose, period, desired_joint_states);
}

controller_interface::return_type AdmittanceRule::update(
  const std::array<double, 6> & /*current_joint_states*/,
  const geometry_msgs::msg::Wrench & /*measured_force*/,
  const geometry_msgs::msg::PoseStamped & /*target_pose*/,
  const geometry_msgs::msg::WrenchStamped & /*target_force*/,
  const rclcpp::Duration & /*period*/,
  std::array<double, 6> /*desired_joint_states*/
)
{
  // TODO(destogl): Implement this update
  //  transform_message_to_control_frame(**input_force_cmd, force_cmd_ctrl_frame_);
  // TODO(destogl) reuse things from other update

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_controller_state(
  control_msgs::msg::AdmittanceControllerState & state_message)
{
  //   state_message.input_force_control_frame = target_force_control_frame_;
  state_message.input_pose_control_frame = target_pose_control_frame_;
  state_message.measured_force = measured_force_;
  state_message.measured_force = measured_force_filtered_;
  state_message.measured_force_control_frame = measured_force_control_frame_;
  state_message.measured_force_endeffector_frame = measured_force_control_frame_;
  state_message.desired_pose = desired_pose_;
  state_message.relative_desired_pose = relative_desired_pose_;

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_current_pose_of_endeffector_frame(geometry_msgs::msg::PoseStamped & pose)
{
  // Get tool frame position - in the future use: IKSolver->getPositionFK(...)
  static geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = endeffector_frame_;
  origin.pose.orientation.w = 1;

  try {
    auto transform = tf_buffer_->lookupTransform(ik_base_frame_, endeffector_frame_, tf2::TimePointZero);
    tf2::doTransform(origin, pose, transform);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_base_frame_ + "' and '" + endeffector_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

}  // namespace admittance_controller
