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
#include "tf2_eigen/tf2_eigen.h"

namespace {  // Utility namespace

// Numerical accuracy checks. Used as deadbands.
static constexpr double WRENCH_EPSILON = 1e-10;
static constexpr double POSE_ERROR_EPSILON = 1e-12;
static constexpr double POSE_EPSILON = 1e-15;

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
  measured_wrench_.header.frame_id = sensor_frame_;

  relative_admittance_pose_.header.frame_id = control_frame_;
  relative_admittance_pose_.child_frame_id = control_frame_;

  reference_joint_deltas_vec_.reserve(6);
  reference_deltas_vec_ik_base_.reserve(6);
  reference_deltas_ik_base_.header.frame_id = ik_base_frame_;
  reference_deltas_ik_base_.child_frame_id = ik_base_frame_;

  identity_transform_.transform.rotation.w = 1;

  relative_desired_joint_state_vec_.reserve(6);

  admittance_rule_calculated_values_.positions.resize(6, 0.0);
  admittance_rule_calculated_values_.velocities.resize(6, 0.0);
  admittance_rule_calculated_values_.accelerations.resize(6, 0.0);
  admittance_rule_calculated_values_.effort.resize(6, 0.0);

  // Initialize IK
  ik_ = std::make_shared<MoveItKinematics>(node, ik_group_name_);

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset()
{
  measured_wrench_ik_base_frame_arr_.fill(0.0);
  reference_pose_ik_base_frame_arr_.fill(0.0);
  current_pose_ik_base_frame_arr_.fill(0.0);
  admittance_velocity_arr_.fill(0.0);
  sum_of_admittance_displacements_.fill(0.0);

  get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);
  reference_pose_from_joint_deltas_ik_base_frame_ = current_pose_ik_base_frame_;

  // "Open-loop" controller uses old desired pose as current pose: current_pose(K) = desired_pose(K-1)
  // Therefore desired pose has to be set before calling *update*-method
  if (open_loop_control_) {
    get_pose_of_control_frame_in_base_frame(admittance_pose_ik_base_frame_);
    convert_message_to_array(admittance_pose_ik_base_frame_, admittance_pose_ik_base_frame_arr_);
  }

  // Initialize ik_tip and tool_frame transformations - those are fixed transformations
  tf2::Stamped<tf2::Transform> tf2_transform;
  try {
    auto transform = tf_buffer_->lookupTransform(ik_tip_frame_, control_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform, tf2_transform);
    ik_tip_to_control_frame_tf_ = tf2_transform;
    control_frame_to_ik_tip_tf_ = tf2_transform.inverse();
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

// Update with target Cartesian pose - the main update method!
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const geometry_msgs::msg::PoseStamped & reference_pose,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // Convert inputs to ik_base frame (assumed stationary)
  transform_message_to_ik_base_frame(reference_pose, reference_pose_ik_base_frame_);

  if (!open_loop_control_) {
    get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);
  } else {
    // In open-loop mode, assume the user's requested pose was exactly achieved
    // TODO(destogl): This will maybe now work when no feed-forward is used
    current_pose_ik_base_frame_ = reference_pose_ik_base_frame_;
  }

  // Convert all data to arrays for simpler calculation
  convert_message_to_array(reference_pose_ik_base_frame_, reference_pose_ik_base_frame_arr_);
  convert_message_to_array(current_pose_ik_base_frame_, current_pose_ik_base_frame_arr_);

  std::array<double, 6> pose_error;

  for (auto i = 0u; i < 6; ++i) {

    if (!open_loop_control_) {
      pose_error[i] = current_pose_ik_base_frame_arr_[i] - reference_pose_ik_base_frame_arr_[i];
    } else {
      // Sum admittance displacement from the previous relative poses
      sum_of_admittance_displacements_[i] += relative_admittance_pose_arr_[i];
      // In open-loop mode, spring force is related to the accumulated "admittance displacement"
      pose_error[i] = sum_of_admittance_displacements_[i];
    }

    if (i >= 3) {
      pose_error[i] = angles::normalize_angle(pose_error[i]);
    }
    if (std::fabs(pose_error[i]) < POSE_ERROR_EPSILON) {
      pose_error[i] = 0.0;
    }
  }

  process_wrench_measurements(measured_wrench);

  calculate_admittance_rule(
    measured_wrench_ik_base_frame_arr_, pose_error, period, relative_admittance_pose_arr_);

  // This works in all cases because not current TF data are used
  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_admittance_pose_arr_, relative_admittance_pose_);

  // Add deltas to previously-desired pose to get the next desired pose
  tf2::doTransform(current_pose_ik_base_frame_, admittance_pose_ik_base_frame_, relative_admittance_pose_);

  return calculate_desired_joint_state(current_joint_state, period, desired_joint_state);
}

// Update from target joint deltas
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const std::array<double, 6> & reference_joint_deltas,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  reference_joint_deltas_vec_.assign(reference_joint_deltas.begin(), reference_joint_deltas.end());

  // Get feed-forward cartesian deltas in the ik_base frame.
  // Since ik_base is MoveIt's working frame, the transform is identity.
  identity_transform_.header.frame_id = ik_base_frame_;
  ik_->update_robot_state(current_joint_state);
  if (!ik_->convert_joint_deltas_to_cartesian_deltas(
      reference_joint_deltas_vec_, identity_transform_, reference_deltas_vec_ik_base_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                 "Conversion of joint deltas to Cartesian deltas failed. Sending current joint"
                 " values to the robot.");
    desired_joint_state = current_joint_state;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  convert_array_to_message(reference_deltas_vec_ik_base_, reference_deltas_ik_base_);

  // Add deltas to previously-desired pose to get the next desired pose
  tf2::doTransform(reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_deltas_ik_base_);

  update(current_joint_state, measured_wrench, reference_pose_from_joint_deltas_ik_base_frame_,
         period, desired_joint_state);

  for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {
    desired_joint_state.positions[i] += reference_joint_deltas[i];
    desired_joint_state.velocities[i] += reference_joint_deltas[i] / period.seconds();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_state*/,
  const geometry_msgs::msg::Wrench & /*measured_wrench*/,
  const geometry_msgs::msg::PoseStamped & /*reference_pose*/,
  const geometry_msgs::msg::WrenchStamped & /*reference_force*/,
  const rclcpp::Duration & /*period*/,
  trajectory_msgs::msg::JointTrajectoryPoint & /*desired_joint_state*/
)
{
  // TODO(destogl): Implement this update
  //  transform_message_to_ik_base_frame(**input_force_cmd, force_cmd_ctrl_frame_);
  // TODO(destogl) reuse things from other update

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_controller_state(
  control_msgs::msg::AdmittanceControllerState & state_message)
{
  //   state_message.input_wrench_control_frame = reference_wrench_control_frame_;
  state_message.input_pose_control_frame = reference_pose_ik_base_frame_;
  state_message.measured_wrench = measured_wrench_;
  state_message.measured_wrench_filtered = measured_wrench_filtered_;
  state_message.measured_wrench_control_frame = measured_wrench_ik_base_frame_;

  // FIXME(destogl): Something is wrong with this transformation - check frames...
  try {
    auto transform = tf_buffer_->lookupTransform(endeffector_frame_, control_frame_, tf2::TimePointZero);
    tf2::doTransform(measured_wrench_ik_base_frame_, measured_wrench_endeffector_frame_, transform);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + endeffector_frame_ + "'.");
  }
  state_message.measured_wrench_endeffector_frame = measured_wrench_endeffector_frame_;

  state_message.admittance_rule_calculated_values = admittance_rule_calculated_values_;

  state_message.current_pose = current_pose_ik_base_frame_;
  state_message.desired_pose = admittance_pose_ik_base_frame_;
  state_message.relative_desired_pose = relative_admittance_pose_;

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_pose_of_control_frame_in_base_frame(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(ik_base_frame_, control_frame_, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation= transform.transform.rotation;
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + ik_base_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void AdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench
)
{
  measured_wrench_.wrench = measured_wrench;
  filter_chain_->update(measured_wrench_, measured_wrench_filtered_);

  // TODO(andyz): This is not flexible to work in other control frames besides ik_base
  transform_message_to_ik_base_frame(measured_wrench_filtered_, measured_wrench_ik_base_frame_);
  convert_message_to_array(measured_wrench_ik_base_frame_, measured_wrench_ik_base_frame_arr_);

  // TODO(destogl): optimize this checks!
  // If at least one measured force is nan set all to 0
  if (std::find_if(measured_wrench_ik_base_frame_arr_.begin(),
    measured_wrench_ik_base_frame_arr_.end(),
    [](const auto value){ return std::isnan(value); }) != measured_wrench_ik_base_frame_arr_.end())
  {
    measured_wrench_ik_base_frame_arr_.fill(0.0);
  }

  // If a force or a torque is very small set it to 0
  for (auto i = 0u; i < measured_wrench_ik_base_frame_arr_.size(); ++i) {
    if (std::fabs(measured_wrench_ik_base_frame_arr_[i]) < WRENCH_EPSILON) {
      measured_wrench_ik_base_frame_arr_[i] = 0.0;
    }
  }
}

void AdmittanceRule::calculate_admittance_rule(
  const std::array<double, 6> & measured_wrench,
  const std::array<double, 6> & pose_error,
  const rclcpp::Duration & period,
  std::array<double, 6> & desired_relative_pose
)
{
  // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
  for (auto i = 0u; i < 6; ++i) {
    if (selected_axes_[i]) {
      // TODO(destogl): check if velocity is measured from hardware
      const double admittance_acceleration = (1 / mass_[i]) * (measured_wrench[i] -
                                             damping_[i] * admittance_velocity_arr_[i] -
                                             stiffness_[i] * pose_error[i]);

      admittance_velocity_arr_[i] += admittance_acceleration * period.seconds();

      // Calculate position
      desired_relative_pose[i] = admittance_velocity_arr_[i] * period.seconds();
      if (std::fabs(desired_relative_pose[i]) < POSE_EPSILON) {
        desired_relative_pose[i] = 0.0;
      }

      // Store data for publishing to state variable
      admittance_rule_calculated_values_.positions[i] = pose_error[i];
      admittance_rule_calculated_values_.velocities[i] = admittance_velocity_arr_[i];
      admittance_rule_calculated_values_.accelerations[i] = admittance_acceleration;
      admittance_rule_calculated_values_.effort[i] = measured_wrench[i];
    }
  }
}

controller_interface::return_type AdmittanceRule::calculate_desired_joint_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  convert_array_to_message(relative_admittance_pose_arr_, relative_admittance_pose_);

  // Since ik_base is MoveIt's working frame, the transform is identity.
  identity_transform_.header.frame_id = ik_base_frame_;

  // Use Jacobian-based IK
  std::vector<double> relative_admittance_pose_vec(relative_admittance_pose_arr_.begin(), relative_admittance_pose_arr_.end());
  ik_->update_robot_state(current_joint_state);
  if (ik_->convert_cartesian_deltas_to_joint_deltas(
    relative_admittance_pose_vec, identity_transform_, relative_desired_joint_state_vec_)){
    for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {
      desired_joint_state.positions[i] =
        current_joint_state.positions[i] + relative_desired_joint_state_vec_[i];
      desired_joint_state.velocities[i] = relative_desired_joint_state_vec_[i] / period.seconds();
    }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the robot.");
      desired_joint_state = current_joint_state;
      std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

}  // namespace admittance_controller
