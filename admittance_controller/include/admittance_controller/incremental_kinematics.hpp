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

#include "Eigen/Core"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/utilities.hpp"

namespace admittance_controller
{

class IncrementalKinematics
{
public:
  /**
   * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
   * It uses the Jacobian from MoveIt.
   */
  IncrementalKinematics(const std::shared_ptr<rclcpp::Node> & node, const std::string & group_name);

  /**
   * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
   * \param delta_x_vec input Cartesian deltas (x, y, z, rx, ry, rz)
   * \param ik_base_to_tip_tf transformation between ik base and ik tip
   * \param delta_theta_vec output vector with joint states
   * \return true if successful
   */
  bool
  convertCartesianDeltasToJointDeltas(const std::vector<double> & delta_x_vec, const geometry_msgs::msg::TransformStamped & ik_base_to_tip_tf, std::vector<double> & delta_theta_vec);

  /**
   * \brief Convert joint delta-theta to Cartesian delta-x, using the Jacobian.
   * \param[in] delta_theta_vec vector with joint states
   * \param[in] ik_base_to_tip_tf transformation between ik base to ik tip
   * \param[out] delta_x_vec  Cartesian deltas (x, y, z, rx, ry, rz)
   * \return true if successful
   */
  bool
  convertJointDeltasToCartesianDeltas(const std::vector<double> &  delta_theta_vec, const geometry_msgs::msg::TransformStamped & ik_base_to_tip_tf, std::vector<double> & delta_x_vec);

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
};

}  // namespace admittance_controller
