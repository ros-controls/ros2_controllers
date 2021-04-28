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

#pragma once

#include <Eigen/Core>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace admittance_controller
{

class IncrementalIKCalculator
{
public:
  /**
   * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
   * It uses the Jacobian from MoveIt.
   */
  IncrementalIKCalculator(std::shared_ptr<rclcpp::Node>& node);

  /**
   * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
   * \param delta_x input Cartesian deltas
   * \param delta_x_frame input name of the delta_x tf frame
   * \param delta_theta output
   * \return true if successful
   */
  bool convertCartesianDeltasToJointDeltas(Eigen::VectorXd delta_x, std::string& delta_x_frame, Eigen::VectorXd& delta_theta);

private:
  // MoveIt setup, required to retrieve the Jacobian
  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::shared_ptr<rclcpp::Node> node_;

  // Pre-allocate for speed
  Eigen::MatrixXd jacobian_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd matrix_s_;
  Eigen::MatrixXd pseudo_inverse_;

  // TF frames
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string moveit_jacobian_frame_;
  geometry_msgs::msg::TransformStamped wrench_to_jacobian_transform_;
};

}  // namespace admittance_controller
