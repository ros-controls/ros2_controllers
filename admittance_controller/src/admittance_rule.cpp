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

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Pose & msg, Type & vector_out)
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

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::PoseStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.pose, vector_out);
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Transform & msg, Type & vector_out)
{
  vector_out[0] = msg.translation.x;
  vector_out[1] = msg.translation.y;
  vector_out[2] = msg.translation.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.rotation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::TransformStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.transform, vector_out);
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::Wrench & msg, Type & vector_out)
{
  vector_out[0] = msg.force.x;
  vector_out[1] = msg.force.y;
  vector_out[2] = msg.force.z;
  vector_out[3] = msg.torque.x;
  vector_out[4] = msg.torque.y;
  vector_out[5] = msg.torque.z;
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::WrenchStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.wrench, vector_out);
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Pose & msg_out)
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

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::PoseStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.pose);
}

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::Wrench & msg_out)
// {
//   msg_out.force.x = vector[0];
//   msg_out.force.y = vector[1];
//   msg_out.force.z = vector[2];
//   msg_out.torque.x = vector[3];
//   msg_out.torque.y = vector[4];
//   msg_out.torque.z = vector[5];
// }

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::WrenchStamped & msg_out)
// {
//   convert_array_to_message(vector, msg_out.wrench);
// }

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

  get_current_pose_of_endeffector_frame(current_pose_);

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
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_force,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // Convert inputs to control frame
  // TODO(andyz): this causes unexpected rotation
  //transform_message_to_control_frame(target_pose, target_pose_control_frame_);
  target_pose_control_frame_ = target_pose;

  if (!hardware_state_has_offset_) {
    get_current_pose_of_endeffector_frame(current_pose_);
    transform_message_to_control_frame(current_pose_, current_pose_control_frame_);
  }
  // TODO(destogl): Can this work properly, when considering offset between states and commands?
//   else {
//     current_pose_control_frame_ = desired_pose_;
//   }

  // Convert all data to arrays for simpler calculation
  convert_message_to_array(target_pose_control_frame_, target_pose_control_frame_arr_);
  convert_message_to_array(current_pose_control_frame_, current_pose_control_frame_arr_);

  std::array<double, 6> pose_error_vec;

  for (auto i = 0u; i < 6; ++i) {
    pose_error_vec[i] = current_pose_control_frame_arr_[i] - target_pose_control_frame_arr_[i];
    if (i >= 3) {
      pose_error_vec[i] = angles::normalize_angle(current_pose_control_frame_arr_[i]) -
      angles::normalize_angle(target_pose_control_frame_arr_[i]);
    }
//     RCLCPP_INFO(rclcpp::get_logger("AdmittanceRule"),
//                 "Pose error [%zu]: %e", pose_error_vec[i]);
  }

  process_force_measurements(measured_force);

  calculate_admittance_rule(measured_force_control_frame_arr_, pose_error_vec, period,
                            relative_desired_pose_arr_);

  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_desired_pose_arr_, relative_desired_pose_);
  tf2::doTransform(current_pose_control_frame_, desired_pose_, relative_desired_pose_);
  transform_ik_tip_to_endeffector_frame(desired_pose_.pose, desired_pose_.pose);

  // Calculate desired Cartesian displacement of the robot
  // TODO: replace this with FK in the long term
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(ik_tip_frame_, ik_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_base_frame_ + "' and '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  // Use Jacobian-based IK
  std::vector<double> relative_desired_pose_vec(relative_desired_pose_arr_.begin(), relative_desired_pose_arr_.end());
  if (ik_->convertCartesianDeltasToJointDeltas(
    relative_desired_pose_vec, transform, relative_desired_joint_state_vec_)){
    for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {
      desired_joint_state.positions[i] = current_joint_state.positions[i] + relative_desired_joint_state_vec_[i];
      desired_joint_state.velocities[i] = relative_desired_joint_state_vec_[i] / period.seconds();
//       RCLCPP_INFO(rclcpp::get_logger("AR"), "joint states [%zu]: %f + %f = %f", i, current_joint_state[i], relative_desired_joint_state_vec_[i], desired_joint_state.positions[i]);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the robot.");
      desired_joint_state = current_joint_state;
      std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
      return controller_interface::return_type::ERROR;
    }

  // TODO(anyone: enable this when enabling use of IK directly
  // transform = tf_buffer_->lookupTransform(endeffector_frame_, ik_base_frame_, tf2::TimePointZero);
  // tf2::doTransform(desired_pose_, ik_input_pose_, transform);
  // ik_input_pose_.pose = transform_endeffector_to_ik_tip_frame(ik_input_pose_);
  // std::vector<double> ik_sol = ik_solver_->getPositionIK (  ); ...

  return controller_interface::return_type::OK;
}

// Update from target joint deltas
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_force,
  const std::array<double, 6> & target_joint_deltas,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  std::vector<double> target_joint_deltas_vec(target_joint_deltas.begin(), target_joint_deltas.end());
  std::vector<double> target_ik_tip_deltas_vec(6);

  geometry_msgs::msg::TransformStamped transform_ik_base_tip;
  try {
    transform_ik_base_tip = tf_buffer_->lookupTransform(ik_base_frame_, ik_tip_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + ik_base_frame_ + "' and '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  // Get cartesian deltas in the IK tip frame
  if (ik_->convertJointDeltasToCartesianDeltas(target_joint_deltas_vec, transform_ik_base_tip, target_ik_tip_deltas_vec)) {
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of joint deltas to Cartesian deltas failed. Sending current joint values to the robot.");
    desired_joint_state = current_joint_state;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  // Get the target pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = ik_tip_frame_;
  target_pose.pose.position.x = target_ik_tip_deltas_vec.at(0);
  target_pose.pose.position.y = target_ik_tip_deltas_vec.at(1);
  target_pose.pose.position.z = target_ik_tip_deltas_vec.at(2);

  tf2::Quaternion q;
  q.setRPY(target_ik_tip_deltas_vec.at(3), target_ik_tip_deltas_vec.at(4), target_ik_tip_deltas_vec.at(5));
  target_pose.pose.orientation.w = q.w();
  target_pose.pose.orientation.x = q.x();
  target_pose.pose.orientation.y = q.y();
  target_pose.pose.orientation.z = q.z();

//   RCLCPP_INFO(rclcpp::get_logger("AdmittanceRule"),
//               "IK-tip deltas: x: %e, y: %e, z: %e, rx: %e, ry: %e, rz: %e",
//               target_ik_tip_deltas_vec.at(0), target_ik_tip_deltas_vec.at(1),
//               target_ik_tip_deltas_vec.at(2), target_ik_tip_deltas_vec.at(3),
//               target_ik_tip_deltas_vec.at(4), target_ik_tip_deltas_vec.at(5)
//              );

  return update(current_joint_state, measured_force, target_pose, period, desired_joint_state);
}

controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_state*/,
  const geometry_msgs::msg::Wrench & /*measured_force*/,
  const geometry_msgs::msg::PoseStamped & /*target_pose*/,
  const geometry_msgs::msg::WrenchStamped & /*target_force*/,
  const rclcpp::Duration & /*period*/,
  trajectory_msgs::msg::JointTrajectoryPoint & /*desired_joint_state*/
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
  state_message.measured_force_filtered = measured_force_filtered_;
  state_message.measured_force_control_frame = measured_force_control_frame_;

  try {
    auto transform = tf_buffer_->lookupTransform(endeffector_frame_, control_frame_, tf2::TimePointZero);
    tf2::doTransform(measured_force_control_frame_, measured_force_endeffector_frame_, transform);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + control_frame_ + "' and '" + endeffector_frame_ + "'.");
  }

  state_message.measured_force_endeffector_frame = measured_force_endeffector_frame_;

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

void AdmittanceRule::process_force_measurements(
  const geometry_msgs::msg::Wrench & measured_forces
)
{
  // Short-circuit if measurement is all zeros or any nan
  const double WRENCH_THRESH = 1e-10;
  if (
    (std::fabs(measured_forces.force.x) < WRENCH_THRESH) &&
    (std::fabs(measured_forces.force.y) < WRENCH_THRESH) &&
    (std::fabs(measured_forces.force.z) < WRENCH_THRESH) &&
    (std::fabs(measured_forces.torque.x) < WRENCH_THRESH) &&
    (std::fabs(measured_forces.torque.y) < WRENCH_THRESH) &&
    (std::fabs(measured_forces.torque.z) < WRENCH_THRESH)
    )
  {
    measured_force_control_frame_arr_.fill(0.0);
    return;
  }
  if (
    std::isnan(measured_forces.force.x) ||
    std::isnan(measured_forces.force.y) ||
    std::isnan(measured_forces.force.z) ||
    std::isnan(measured_forces.torque.x) ||
    std::isnan(measured_forces.torque.y) ||
    std::isnan(measured_forces.torque.z)
    )
  {
    measured_force_control_frame_arr_.fill(0.0);
    return;
  }

  // get current states, and transform those into controller frame
  measured_force_.wrench = measured_forces;
  try {
    auto transform = tf_buffer_->lookupTransform(fixed_world_frame_,  measured_force_.header.frame_id, tf2::TimePointZero);
    auto transform_back = tf_buffer_->lookupTransform(measured_force_.header.frame_id, fixed_world_frame_, tf2::TimePointZero);

    geometry_msgs::msg::WrenchStamped measured_force_transformed;
    tf2::doTransform(measured_force_, measured_force_transformed, transform);

    geometry_msgs::msg::Vector3Stamped cog_transformed;

    for (const auto & params : gravity_compensation_params_) {
      auto transform_cog = tf_buffer_->lookupTransform(fixed_world_frame_,  params.cog_.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(params.cog_, cog_transformed, transform_cog);

//      measured_force_transformed.wrench.force.z += params.force_;
//      measured_force_transformed.wrench.torque.x += (params.force_ * cog_transformed.vector.y);
//      measured_force_transformed.wrench.torque.y -= (params.force_ * cog_transformed.vector.x);
    }

    tf2::doTransform(measured_force_transformed, measured_force_filtered_, transform_back);

  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + fixed_world_frame_ + "' and '" + measured_force_.header.frame_id + "' or '<a cog frame>'.");
    // If transform error just use measured force
    measured_force_filtered_ = measured_force_;
  }

  transform_message_to_control_frame(measured_force_filtered_, measured_force_control_frame_);

  convert_message_to_array(measured_force_control_frame_, measured_force_control_frame_arr_);

  // If at least one measured force is nan set all to 0
  if (std::find_if(measured_force_control_frame_arr_.begin(), measured_force_control_frame_arr_.end(), [](const auto value){ return std::isnan(value); }) != measured_force_control_frame_arr_.end()) {
    measured_force_control_frame_arr_.fill(0.0);
  }
}

void AdmittanceRule::calculate_admittance_rule(
  const std::array<double, 6> & measured_force,
  const std::array<double, 6> & pose_error,
  const rclcpp::Duration & period,
  std::array<double, 6> & desired_relative_pose
)
{
  // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
  for (auto i = 0u; i < 6; ++i) {
    if (selected_axes_[i]) {

      RCLCPP_INFO_STREAM(rclcpp::get_logger("AR"), measured_force[i]);

      // TODO(destogl): check if velocity is measured from hardware
      const double acceleration = 1 / mass_[i] *
      (measured_force[i] - damping_[i] * desired_velocity_arr_[i] -
      stiffness_[i] * pose_error[i]);

      desired_velocity_arr_[i] += acceleration * period.seconds();

      desired_relative_pose[i] = (desired_velocity_previous_arr_[i] + desired_velocity_arr_[i]) * 0.5 * period.seconds();

      desired_acceleration_previous_arr_[i] = acceleration;
      desired_velocity_previous_arr_[i] = desired_velocity_arr_[i];

      // RCLCPP_INFO(rclcpp::get_logger("AR"), "Pose error, acceleration, desired velocity, relative desired pose [%zu]: (%e - D*%e - S*%e = %e)", i, measured_force[i], desired_velocity_arr_[i], pose_error , acceleration);
    }
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("AR"), "---");
}

controller_interface::return_type AdmittanceRule::calculate_desired_joint_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_desired_pose_arr_, relative_desired_pose_);
  tf2::doTransform(current_pose_control_frame_, desired_pose_, relative_desired_pose_);
  transform_ik_tip_to_endeffector_frame(desired_pose_.pose, desired_pose_.pose);

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

  // Use Jacobian-based IK
  std::vector<double> relative_desired_pose_vec(relative_desired_pose_arr_.begin(), relative_desired_pose_arr_.end());
  if (ik_->convertCartesianDeltasToJointDeltas(
    relative_desired_pose_vec, transform, relative_desired_joint_state_vec_)){
    for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {
      desired_joint_state.positions[i] = current_joint_state.positions[i] + relative_desired_joint_state_vec_[i];
      desired_joint_state.velocities[i] = relative_desired_joint_state_vec_[i] / period.seconds();
      //       RCLCPP_INFO(rclcpp::get_logger("AR"), "joint states [%zu]: %f + %f = %f", i, current_joint_state.positions[i], relative_desired_joint_state_vec_[i], desired_joint_state.positions[i]);
    }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the robot.");
      desired_joint_state = current_joint_state;
      std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
      return controller_interface::return_type::ERROR;
    }

    // TODO(anyone: enable this when enabling use of IK directly
    // transform = tf_buffer_->lookupTransform(endeffector_frame_, ik_base_frame_, tf2::TimePointZero);
    // tf2::doTransform(desired_pose_, ik_input_pose_, transform);
    // ik_input_pose_.pose = transform_endeffector_to_ik_tip_frame(ik_input_pose_);
    // std::vector<double> ik_sol = ik_solver_->getPositionIK (  ); ...

    return controller_interface::return_type::OK;
}

}  // namespace admittance_controller
