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
/// \author: Andy Zelenak

#include "admittance_controller/moveit_kinematics.hpp"

#include "tf2_eigen/tf2_eigen.h"

namespace admittance_controller
{
MoveItKinematics::MoveItKinematics(const std::shared_ptr<rclcpp::Node> & node, const std::string & group_name) : node_(node)
{
  // TODO(andyz): Parameterize robot description and joint group
  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr =
      std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader(node_, "robot_description", false /* do not load kinematics plugins */));
  const moveit::core::RobotModelPtr& kinematic_model = model_loader_ptr->getModel();
  // TODO(andyz): joint_model_group_ is a raw pointer. Is it thread safe? (Denis: there should not be multi-threading here)
  joint_model_group_ = kinematic_model->getJointModelGroup(group_name);
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);

  // By default, the MoveIt Jacobian frame is the last link
}

bool MoveItKinematics::convert_cartesian_deltas_to_joint_deltas(std::vector<double> & delta_x_vec, const geometry_msgs::msg::TransformStamped & control_frame_to_ik_base, std::vector<double> & delta_theta_vec)
{
  // see here for this conversion: https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
  Eigen::VectorXd delta_x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(&delta_x_vec[0], delta_x_vec.size());

  // Transform delta_x to the moveit_jacobian_frame
  // TODO: replace when this PR to tf2_eigen is merged
  // https://github.com/ros2/geometry2/pull/406
  try
  {
    // 4x4 transformation matrix
    const Eigen::Isometry3d affine_transform = tf2::transformToEigen(control_frame_to_ik_base);

    // Build the 6x6 transformation matrix
    Eigen::MatrixXd twist_transform(6,6);
    // upper left 3x3 block is the rotation part
    twist_transform.block(0,0,3,3) = affine_transform.rotation();
    // upper right 3x3 block is all zeros
    twist_transform.block(0,3,3,3) = Eigen::MatrixXd::Zero(3,3);
    // lower left 3x3 block is tricky. See https://core.ac.uk/download/pdf/154240607.pdf
    Eigen::MatrixXd pos_vector_3x3(3,3);
    pos_vector_3x3(0,0) = 0;  pos_vector_3x3(0,1) = -affine_transform.translation().z();  pos_vector_3x3(0,2) = affine_transform.translation().y();
    pos_vector_3x3(1, 0) = affine_transform.translation().z();  pos_vector_3x3(1,1) = 0;  pos_vector_3x3(1,2) = -affine_transform.translation().x();
    pos_vector_3x3(2, 0) = -affine_transform.translation().y();  pos_vector_3x3(2,1) = affine_transform.translation().x();  pos_vector_3x3(1,2) = 0;
    twist_transform.block(3,0,3,3) = pos_vector_3x3 * affine_transform.rotation();
    // lower right 3x3 block is the rotation part
    twist_transform.block(3,3,3,3) = affine_transform.rotation();

    delta_x = twist_transform * delta_x;
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Transformation of twist failed.");
    return false;
  }

  // Multiply with the pseudoinverse to get delta_theta
  jacobian_ = kinematic_state_->getJacobian(joint_model_group_);
  // TODO(andyz): the SVD method would be more stable near singularities
  // (or do what Olivier suggested: https://github.com/ros-controls/ros2_controllers/pull/173#discussion_r627936628)
  pseudo_inverse_ = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse();
  Eigen::VectorXd  delta_theta = pseudo_inverse_ * delta_x;

  std::vector<double> delta_theta_v(&delta_theta[0], delta_theta.data() + delta_theta.cols() * delta_theta.rows());
  delta_theta_vec = delta_theta_v;

  return true;
}

bool MoveItKinematics::convert_joint_deltas_to_cartesian_deltas(std::vector<double> &  delta_theta_vec, const geometry_msgs::msg::TransformStamped & tf_ik_base_to_desired_cartesian_frame, std::vector<double> & delta_x_vec)
{
  // see here for this conversion: https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
  Eigen::VectorXd delta_theta = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(&delta_theta_vec[0], delta_theta_vec.size());

  // Multiply with the Jacobian to get delta_x
  jacobian_ = kinematic_state_->getJacobian(joint_model_group_);
  // delta_x will be in the working frame of MoveIt (ik_base frame)
  Eigen::VectorXd delta_x = jacobian_ * delta_theta;

  // Transform delta_x to the tip frame
  // TODO: replace when this PR to tf2_eigen is merged
  // https://github.com/ros2/geometry2/pull/406
  try
  {
    // 4x4 transformation matrix
    const Eigen::Isometry3d affine_transform = tf2::transformToEigen(tf_ik_base_to_desired_cartesian_frame);

    // Build the 6x6 transformation matrix
    Eigen::MatrixXd twist_transform(6,6);
    // upper left 3x3 block is the rotation part
    twist_transform.block(0,0,3,3) = affine_transform.rotation();
    // upper right 3x3 block is all zeros
    twist_transform.block(0,3,3,3) = Eigen::MatrixXd::Zero(3,3);
    // lower left 3x3 block is tricky. See https://core.ac.uk/download/pdf/154240607.pdf
    Eigen::MatrixXd pos_vector_3x3(3,3);
    pos_vector_3x3(0,0) = 0;  pos_vector_3x3(0,1) = -affine_transform.translation().z();  pos_vector_3x3(0,2) = affine_transform.translation().y();
    pos_vector_3x3(1, 0) = affine_transform.translation().z();  pos_vector_3x3(1,1) = 0;  pos_vector_3x3(1,2) = -affine_transform.translation().x();
    pos_vector_3x3(2, 0) = -affine_transform.translation().y();  pos_vector_3x3(2,1) = affine_transform.translation().x();  pos_vector_3x3(1,2) = 0;
    twist_transform.block(3,0,3,3) = pos_vector_3x3 * affine_transform.rotation();
    // lower right 3x3 block is the rotation part
    twist_transform.block(3,3,3,3) = affine_transform.rotation();

    delta_x = twist_transform * delta_x;
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Transformation of twist failed.");
    return false;
  }

  std::vector<double> delta_x_v(&delta_x[0], delta_x.data() + delta_x.cols() * delta_x.rows());
  delta_x_vec = delta_x_v;

  return true;
}

Eigen::Isometry3d MoveItKinematics::get_link_transform(const std::string& link_name, const trajectory_msgs::msg::JointTrajectoryPoint & joint_state)
{
  update_robot_state(joint_state);

  return kinematic_state_->getGlobalLinkTransform(link_name);
}

}  // namespace admittance_controller
