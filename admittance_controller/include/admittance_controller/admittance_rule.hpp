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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_ros/buffer.h>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>
#include "tf2_kdl/tf2_kdl/tf2_kdl.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "control_toolbox/filters.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "admittance_controller_parameters.hpp"

// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"


namespace admittance_controller {

  struct Transforms {
    Eigen::Isometry3d base_control_;
    Eigen::Isometry3d  base_ft_;
    Eigen::Isometry3d  base_tip_;
    Eigen::Isometry3d  world_base_;
    Eigen::Isometry3d  base_sensor_;
    Eigen::Isometry3d  base_cog_;
  };

  struct AdmittanceState {
    AdmittanceState() = default;

    AdmittanceState(size_t num_joints) {
      admittance_velocity.setZero();
      admittance_acceleration.setZero();
      damping.setZero();
      mass.setOnes();
      mass_inv.setZero();
      stiffness.setZero();
      selected_axes.setZero();
      current_joint_pos = Eigen::VectorXd::Zero(num_joints);
      joint_pos = Eigen::VectorXd::Zero(num_joints);
      joint_vel = Eigen::VectorXd::Zero(num_joints);
      joint_acc = Eigen::VectorXd::Zero(num_joints);
    }

    Eigen::VectorXd current_joint_pos;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd joint_acc;
    Eigen::Matrix<double, 6, 1> damping;
    Eigen::Matrix<double, 6, 1> mass;
    Eigen::Matrix<double, 6, 1> mass_inv;
    Eigen::Matrix<double, 6, 1> selected_axes;
    Eigen::Matrix<double, 6, 1> stiffness;
    Eigen::Matrix<double, 6, 1> wrench_base;
    Eigen::Matrix<double, 6, 1> admittance_acceleration;
    Eigen::Matrix<double, 6, 1> admittance_velocity;
    Eigen::Isometry3d admittance_position;
    Eigen::Matrix<double, 3, 3> rot_base_control;
    Eigen::Isometry3d ref_trans_base_ft;
    std::string ft_sensor_frame;
  };

  class AdmittanceRule {
  public:
    AdmittanceRule(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &lifecycl_node) {
      parameter_handler_ = std::make_shared<admittance_controller::ParamListener>(
          lifecycl_node->get_node_parameters_interface());
      parameters_ = parameter_handler_->get_params();
      num_joints_ = parameters_.joints.size();
      admittance_state_ = AdmittanceState(num_joints_);
      reset(num_joints_);
    }

    controller_interface::return_type
    configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, size_t num_joint);

    controller_interface::return_type reset(size_t num_joints);

    /**
     * Calculate all transforms needed for admittance control using the loader kinematics plugin. If the transform does
     * not exist in the kinematics model, then TF will be used for lookup. The return value is true if all transformation
     * are calculated without an error
     * \param[in] current_joint_state
     * \param[in] reference_joint_state
     * \param[in] success
     */
    bool get_all_transforms(const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
                            const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
                            const AdmittanceState &admittance_state);

    /**
    * Updates parameter_ struct if any parameters have changed since last update. Parameter dependent Eigen field
    * members (ee_weight_, cog_, mass_, mass_inv_ stiffness, selected_axes, damping_) are also updated
    */
    void apply_parameters_update();

    /**
     * Calculate 'desired joint states' based on the 'measured force', 'reference joint state', and
     * 'current_joint_state'.
     *
     * \param[in] current_joint_state
     * \param[in] measured_wrench
     * \param[in] reference_joint_state
     * \param[in] period
     * \param[out] desired_joint_state
     */
    controller_interface::return_type update(
        const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
        const geometry_msgs::msg::Wrench &measured_wrench,
        const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
        const rclcpp::Duration &period,
        trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_states);

    /**
     * Fill fields of `state_message` from current admittance controller state.
     *
     * \param[in] state_message
     */
    const control_msgs::msg::AdmittanceControllerState & get_controller_state();

  public:
    // admittance config parameters
    std::shared_ptr<admittance_controller::ParamListener> parameter_handler_;
    admittance_controller::Params parameters_;
  protected:
    /**
     * Calculate the admittance rule from given the robot's current joint angles. the control frame rotation, the
     * current force torque transform, the reference force torque transform, the force torque sensor frame id,
     * the time delta, and the current admittance controller state. The admittance controller state input is updated
     * with the new calculated values. A boolean value is returned indicating if any of the kinematics plugin calls
     * failed.
     * \param[in] current_joint_positions
     * \param[in] base_control
     * \param[in] base_ft
     * \param[in] ref_base_ft
     * \param[in] ft_sensor_frame
     * \param[in] dt
     * \param[in] admittance_state
     * \param[out] success
     */
    bool calculate_admittance_rule(AdmittanceState &admittance_state, double dt);

    /**
     * Updates internal estimate of wrench in world frame `wrench_world_` given the new measurement. The `wrench_world_`
     * estimate includes gravity compensation
     * \param[in] measured_wrench
     * \param[in] sensor_rot
     * \param[in] cog_rot
     */
    void process_wrench_measurements(
        const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 3, 3> &sensor_rot,
        const Eigen::Matrix<double, 3, 3> &cog_rot
    );

    template<typename T1, typename T2>
    void vec_to_eigen(const std::vector<T1> &data, T2 &matrix);

    template<typename T1, typename T2>
    void eigen_to_vec(const T2 &matrix, std::vector<T1> &data);

    // number of robot joint
    size_t num_joints_;

    // Kinematics interface plugin loader
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterfaceBase>> kinematics_loader_;
    std::unique_ptr<kinematics_interface::KinematicsInterfaceBase> kinematics_;

    // measured wrench in ft frame
//    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> measured_wrench_;

    // filtered wrench in world frame
    Eigen::Matrix<double, 6, 1> wrench_world_;

    // admittance controllers internal state
    AdmittanceState admittance_state_;

    // transforms needed for admittance update
    Transforms trans_;
    Transforms trans_ref_;

    // position of center of gravity in cog_frame
    Eigen::Vector3d cog_;

    // force applied to sensor due to weight of end effector
    Eigen::Vector3d ee_weight;

    // ROS
    control_msgs::msg::AdmittanceControllerState state_message_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
