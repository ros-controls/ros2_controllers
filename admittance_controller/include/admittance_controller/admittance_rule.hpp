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
#include "config/admittance_struct.h"

// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"


namespace {  // Utility namespace

// Numerical accuracy checks. Used as deadbands.
  const double ROT_AXIS_EPSILON = 1e-6;


}  // utility namespace

namespace admittance_controller {

  class AdmittanceRule {
  public:
    AdmittanceRule() {
      parameters_ = &parameters_copy;
    }

    controller_interface::return_type
    configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node, int num_joint);

    controller_interface::return_type reset();

    /**
     * Calculate 'desired joint states' based on the 'measured force' and 'reference joint state'.
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

    void get_controller_state(control_msgs::msg::AdmittanceControllerState &state_message);


  public:
    // admittance config parameters
    std::shared_ptr<admittance_struct_parameters::admittance_struct> parameter_handler_;
    admittance_struct_parameters::admittance_struct::params *parameters_;
    admittance_struct_parameters::admittance_struct::params parameters_copy;

  protected:
    /**
     * All values are in the controller frame
     */
    void calculate_admittance_rule(
        const Eigen::Matrix<double, 3, 2> &wrench,
        const Eigen::Matrix<double, 3, 2> &desired_vel,
        const double dt
    );

    void process_wrench_measurements(
        const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 3, 3> &sensor_rot,
        const Eigen::Matrix<double, 3, 3> &cog_rot
    );

    Eigen::Vector3d get_rotation_axis(const Eigen::Matrix3d &R) const;

    void convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &positions,
                                                  const Eigen::Matrix<double, 3, 2> &cartesian_delta,
                                                  const std::string &link_name,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_delta, bool &success);

    Eigen::Matrix<double, 3, 2> convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &positions,
                                                                         const std::vector<double> &joint_delta,
                                                                         const std::string &link_name,
                                                                         bool &success);

    void normalize_rotation(Eigen::Matrix<double, 3, 3, Eigen::ColMajor> &R);

    Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
    get_transform(const std::vector<double> &positions, const std::string &link_name, bool external, bool &success);

    void eigen_to_msg(const Eigen::Matrix<double, 3, 2> &wrench, const std::string &frame_id,
                      geometry_msgs::msg::WrenchStamped &wrench_msg);

    template<typename T1, typename T2>
    void vec_to_eigen(const std::vector<T1> data, T2 &matrix);

    // Kinematics interface plugin loader
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> kinematics_loader_;
    std::unique_ptr<kinematics_interface::KinematicsBaseClass> kinematics_;

    // number of robot joint
    int num_joints_;

    // buffers to pass data to kinematics interface
    std::vector<double> transform_buffer_vec_;
    std::vector<double> joint_buffer_vec_;
    std::vector<double> cart_buffer_vec_;
    std::vector<double> jacobian_buffer_vec_;

    // admittance controller values
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> admittance_acceleration_;
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> admittance_velocity_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> admittance_position_;

    // transforms
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> ee_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> reference_ft_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> sensor_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> control_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> cog_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> world_transform_;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> ft_transform_;

    // rotations
    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> ee_rot_;
    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> control_rot_;
    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> sensor_rot_;
    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> cog_rot_;
    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> world_rot_;

    // external force
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> wrench_world_;
    Eigen::Matrix<double, 3, 2, Eigen::ColMajor> measured_wrench_;
    // position of center of gravity in cog_frame
    Eigen::Vector3d cog_;
    // force applied to sensor due to weight of end effector
    Eigen::Vector3d ee_weight;

    // admittance controller values in joint space
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_pos_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_vel_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_acc_;

    Eigen::Matrix<double,6,1> damping_;
    Eigen::Matrix<double,6,1> mass_;
    Eigen::Matrix<double,6,1> mass_inv_;
    Eigen::Matrix<double,6,1> selected_axes_;
    Eigen::Matrix<double,6,1> stiffness_;

    // jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> identity;

    // ROS
    trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;
    control_msgs::msg::AdmittanceControllerState state_message_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
