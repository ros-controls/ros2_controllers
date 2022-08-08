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
#include <tf2_ros/buffer.h>
#include "tf2_kdl/tf2_kdl/tf2_kdl.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "control_toolbox/filters.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "config/admittance_controller_parameters.hpp"

// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"


namespace utility {
// Numerical accuracy checks. Used as deadbands.
  const double ROT_AXIS_EPSILON = 1e-12;
}  // utility namespace

namespace admittance_controller {

  struct Transforms {
    Transforms() {
      base_control_.setIdentity();
      base_ft_.setIdentity();
      base_tip_.setIdentity();
      world_base_.setIdentity();
      base_sensor_.setIdentity();
      base_cog_.setIdentity();
      base_desired_ft_.setIdentity();
    }

    Eigen::Matrix<double, 4, 4> base_control_;
    Eigen::Matrix<double, 4, 4> base_ft_;
    Eigen::Matrix<double, 4, 4> base_desired_ft_;
    Eigen::Matrix<double, 4, 4> base_tip_;
    Eigen::Matrix<double, 4, 4> world_base_;
    Eigen::Matrix<double, 4, 4> base_sensor_;
    Eigen::Matrix<double, 4, 4> base_cog_;
  };

  struct AdmittanceState {
    AdmittanceState() = default;

    AdmittanceState(int num_joints) {
      admittance_position_.setIdentity();
      admittance_velocity_.setZero();
      admittance_acceleration_.setZero();
      damping_.setZero();
      mass_.setOnes();
      mass_inv_.setZero();
      stiffness_.setZero();
      selected_axes_.setZero();
      joint_pos_ = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>::Zero(num_joints);
      joint_vel_ = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>::Zero(num_joints);
      joint_acc_ = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>::Zero(num_joints);
    }

    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_pos_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_vel_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_acc_;
    Eigen::Matrix<double, 6, 1> damping_;
    Eigen::Matrix<double, 6, 1> mass_;
    Eigen::Matrix<double, 6, 1> mass_inv_;
    Eigen::Matrix<double, 6, 1> selected_axes_;
    Eigen::Matrix<double, 6, 1> stiffness_;
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> wrench_base;
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> admittance_acceleration_;
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> admittance_velocity_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> admittance_position_;
  };

  class AdmittanceRule {
  public:
    AdmittanceRule(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &lifecycl_node) {
      parameter_handler_ = std::make_shared<admittance_controller::ParamListener>(
          lifecycl_node->get_node_parameters_interface());
      parameters_ = parameter_handler_->get_params();
      num_joints_ = parameters_.joints.size();
      admittance_state_ = AdmittanceState(num_joints_);

    }

    controller_interface::return_type
    configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, int num_joint);

    controller_interface::return_type reset(int num_joints);

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
    void get_controller_state(control_msgs::msg::AdmittanceControllerState &state_message);

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
    bool calculate_admittance_rule(const std::vector<double> &current_joint_positions,
                                   const Eigen::Matrix<double, 3, 3, Eigen::ColMajor> &base_control,
                                   Eigen::Matrix<double, 4, 4, Eigen::ColMajor> base_ft,
                                   Eigen::Matrix<double, 4, 4, Eigen::ColMajor> ref_base_ft,
                                   Eigen::Matrix<double, 4, 4, Eigen::ColMajor> base_desired_ft,
                                   const std::string &ft_sensor_frame,
                                   double dt,
                                   AdmittanceState &admittance_state);

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

    /**
     * Returns the axis of rotation of a given rotation matrix
     * \param[in] R
     * \param[out] rotation_axis
     */
    Eigen::Vector3d get_rotation_axis(const Eigen::Matrix3d &R) const;

    /**
    * Normalizes given rotation matrix `R`
    * \param[in] R
    */
    static void normalize_rotation(Eigen::Matrix<double, 3, 3, Eigen::ColMajor> &R);

    bool convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &positions,
                                                  const Eigen::Matrix<double, 3, 2> &cartesian_delta,
                                                  const std::string &link_name,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_delta);

    bool convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &positions,
                                                  const std::vector<double> &joint_delta,
                                                  const std::string &link_name,
                                                  Eigen::Matrix<double, 3, 2> &cartesian_delta);


    /**
    * Calculates the transform from the specified link to the robot's base link at the given joint positions. If
     * `external` is set to true, then the link transform with will queried using TF lookup.
    * \param[in] positions
    * \param[in] link_name
    * \param[in] external
    * \param[in] transform
    */
    bool get_transform(const std::vector<double> &positions, const std::string &link_name, bool external,
                       Eigen::Matrix<double, 4, 4, Eigen::ColMajor> &transform);

    void eigen_to_msg(const Eigen::Matrix<double, 3, 2> &wrench, const std::string &frame_id,
                      geometry_msgs::msg::WrenchStamped &wrench_msg);

    template<typename T1, typename T2>
    void vec_to_eigen(const std::vector<T1>& data, T2 &matrix);

    template<typename T1, typename T2>
    void eigen_to_vec(const T2 &matrix, std::vector<T1>& data);

    // number of robot joint
    int num_joints_;

    // Kinematics interface plugin loader
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> kinematics_loader_;
    std::unique_ptr<kinematics_interface::KinematicsBaseClass> kinematics_;

    // buffers to pass data to kinematics interface
    std::vector<double> transform_buffer_vec_;
    std::vector<double> joint_buffer_vec_;
    std::vector<double> cart_buffer_vec_;

    // measured wrench in ft frame
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> measured_wrench_;
    // filtered wrench in world frame
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> wrench_world_;

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
    trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;
    control_msgs::msg::AdmittanceControllerState state_message_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
