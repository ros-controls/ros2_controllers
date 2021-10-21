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

#include "tf2_eigen/tf2_eigen.hpp"

constexpr auto ROS_LOG_THROTTLE_PERIOD = std::chrono::milliseconds(3000).count();
// TODO: Parameterize singularity thresholds
constexpr double LOWER_SINGULARITY_THRESHOLD = 20.;
constexpr double APPROACHING_STOP_SINGULARITY_THRESHOLD = 60.;
constexpr double HARD_STOP_SINGULARITY_THRESHOLD = 120.;

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

bool MoveItKinematics::convert_cartesian_deltas_to_joint_deltas(
  std::vector<double> & delta_x_vec,
  const geometry_msgs::msg::TransformStamped & control_frame_to_ik_base,
  std::vector<double> & delta_theta_vec)
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
  // TODO(andyz): consider what Olivier suggested: https://github.com/ros-controls/ros2_controllers/pull/173#discussion_r627936628
  Eigen::JacobiSVD<Eigen::MatrixXd> svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  matrix_s_ = svd.singularValues().asDiagonal();
  pseudo_inverse_ = svd.matrixV() * matrix_s_.inverse() * svd.matrixU().transpose();

  Eigen::VectorXd  delta_theta = pseudo_inverse_ * delta_x;
  delta_theta *= velocityScalingFactorForSingularity(delta_x, svd, pseudo_inverse_);

  std::vector<double> delta_theta_v(&delta_theta[0], delta_theta.data() + delta_theta.cols() * delta_theta.rows());
  delta_theta_vec = delta_theta_v;

  return true;
}

bool MoveItKinematics::convert_joint_deltas_to_cartesian_deltas(
  std::vector<double> &  delta_theta_vec,
  const geometry_msgs::msg::TransformStamped & tf_ik_base_to_desired_cartesian_frame,
  std::vector<double> & delta_x_vec)
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

Eigen::Isometry3d MoveItKinematics::get_link_transform(
  const std::string& link_name, const trajectory_msgs::msg::JointTrajectoryPoint & joint_state)
{
  update_robot_state(joint_state);

  return kinematic_state_->getGlobalLinkTransform(link_name);
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double MoveItKinematics::velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                                       const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                       const Eigen::MatrixXd& pseudo_inverse)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(num_dimensions - 1);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // TODO: Remove or switch to DEBUG
  RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 50000., "Singularity condition number is " << ini_condition);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(num_dimensions);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse * delta_x;
  kinematic_state_->setJointGroupPositions(joint_model_group_, new_theta);
  Eigen::MatrixXd new_jacobian = kinematic_state_->getJacobian(joint_model_group_);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==> decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  double upper_threshold = dot > 0 ? APPROACHING_STOP_SINGULARITY_THRESHOLD : HARD_STOP_SINGULARITY_THRESHOLD;
  // Ramp velocity down linearly when the Jacobian condition is between lower_singularity_threshold and
  // hard_stop_singularity_threshold, and we're moving towards the singularity
  if ((ini_condition > LOWER_SINGULARITY_THRESHOLD) &&
      (ini_condition < upper_threshold))
  {
    velocity_scale =
        1. - (ini_condition - LOWER_SINGULARITY_THRESHOLD) /
                 (upper_threshold - LOWER_SINGULARITY_THRESHOLD);
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD, "Close to a singularity, decelerating");
  }

  // Very close to singularity, so halt.
  else if (ini_condition > upper_threshold)
  {
    velocity_scale = 0;
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD, "Very close to a singularity, emergency stop");
  }

  return velocity_scale;
}

}  // namespace admittance_controller
