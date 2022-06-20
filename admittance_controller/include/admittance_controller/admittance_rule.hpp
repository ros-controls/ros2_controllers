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
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "admittance_controller/parameter_handler.hpp"
#include "controller_interface/controller_interface.hpp"
#include "control_toolbox/filters.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"


// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"

namespace {  // Utility namespace

// Numerical accuracy checks. Used as deadbands.
  static constexpr double WRENCH_EPSILON = 1e-10;
  static constexpr double POSE_ERROR_EPSILON = 1e-12;
  static constexpr double POSE_EPSILON = 1e-15;


}  // utility namespace

namespace admittance_controller {

  class AdmittanceParameters : public control_toolbox::ParameterHandler {
  public:
    AdmittanceParameters() : control_toolbox::ParameterHandler("", 7, 0, 28, 7) {
      add_string_parameter("IK.base", false);
      add_string_parameter("IK.group_name", false);
      add_string_parameter("IK.plugin_name", false);
      add_string_parameter("control_frame", true);
      add_string_parameter("sensor_frame", false);
      add_string_parameter("end_effector_name", false);
      add_string_parameter("CoG.frame", false);

      add_bool_parameter("open_loop_control", true);
      add_bool_parameter("enable_parameter_update_without_reactivation", false);

      add_bool_parameter("admittance.selected_axes.x", true);
      add_bool_parameter("admittance.selected_axes.y", true);
      add_bool_parameter("admittance.selected_axes.z", true);
      add_bool_parameter("admittance.selected_axes.rx", true);
      add_bool_parameter("admittance.selected_axes.ry", true);
      add_bool_parameter("admittance.selected_axes.rz", true);

      add_double_parameter("admittance.mass.x", true);
      add_double_parameter("admittance.mass.y", true);
      add_double_parameter("admittance.mass.z", true);
      add_double_parameter("admittance.mass.rx", true);
      add_double_parameter("admittance.mass.ry", true);
      add_double_parameter("admittance.mass.rz", true);
      add_double_parameter("admittance.stiffness.x", true);
      add_double_parameter("admittance.stiffness.y", true);
      add_double_parameter("admittance.stiffness.z", true);
      add_double_parameter("admittance.stiffness.rx", true);
      add_double_parameter("admittance.stiffness.ry", true);
      add_double_parameter("admittance.stiffness.rz", true);
      add_double_parameter("admittance.damping.x", true);
      add_double_parameter("admittance.damping.y", true);
      add_double_parameter("admittance.damping.z", true);
      add_double_parameter("admittance.damping.rx", true);
      add_double_parameter("admittance.damping.ry", true);
      add_double_parameter("admittance.damping.rz", true);
      add_double_parameter("admittance.damping_ratio.x", true);
      add_double_parameter("admittance.damping_ratio.y", true);
      add_double_parameter("admittance.damping_ratio.z", true);
      add_double_parameter("admittance.damping_ratio.rx", true);
      add_double_parameter("admittance.damping_ratio.ry", true);
      add_double_parameter("admittance.damping_ratio.rz", true);

      add_double_parameter("CoG.x", false);
      add_double_parameter("CoG.y", false);
      add_double_parameter("CoG.z", false);
      add_double_parameter("CoG.force", false);
    }

    bool check_if_parameters_are_valid() override {
      bool ret = true;

      // Check if any string parameter is empty
      ret = !empty_parameter_in_list(string_parameters_);

      int index = 0;
      int offset_index_bool = 2;
      // check if parameters are all properly set for selected axes
      for (size_t i = 0; i < 6; ++i) {
        if (bool_parameters_[offset_index_bool + i].second) {
          // check mass parameters
          index = i;
          if (std::isnan(double_parameters_[index].second)) {
            RCUTILS_LOG_ERROR_NAMED(
                logger_name_.c_str(),
                "Parameter '%s' has to be set", double_parameters_[index].first.name.c_str());
            ret = false;
          }
          // Check stiffness parameters
          index = i + 6;
          if (std::isnan(double_parameters_[index].second)) {
            RCUTILS_LOG_ERROR_NAMED(
                logger_name_.c_str(),
                "Parameter '%s' has to be set", double_parameters_[index].first.name.c_str());
            ret = false;
          }
          // Check damping or damping_ratio parameters
          index = i + 12;
          if (std::isnan(double_parameters_[index].second) &&
              std::isnan(double_parameters_[index + 6].second)) {
            RCUTILS_LOG_ERROR_NAMED(
                logger_name_.c_str(),
                "Either parameter '%s' of '%s' has to be set",
                double_parameters_[index].first.name.c_str(),
                double_parameters_[index + 6].first.name.c_str()
            );
            ret = false;
          }
        }
      }

      return ret;
    }

    /**
     * Conversion to damping when damping_ratio (zeta) parameter is used.
     * Using formula: D = damping_ratio * 2 * sqrt( M * S )
     */
    void convert_damping_ratio_to_damping() {
      for (auto i = 0ul; i < damping_ratio_.size(); ++i) {
        if (!std::isnan(damping_ratio_[i])) {
          damping_[i] = damping_ratio_[i] * 2 * sqrt(mass_[i] * stiffness_[i]);
        } else {
          RCUTILS_LOG_DEBUG_NAMED(
              logger_name_.c_str(),
              "Damping ratio for axis %zu not used because it is NaN.", i);
        }
      }
    }

    void update_storage() override {
      ik_base_frame_ = string_parameters_[0].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "IK Base frame: %s", ik_base_frame_.c_str());
      ik_group_name_ = string_parameters_[1].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "IK group name frame: %s", ik_group_name_.c_str());
      ik_plugin_name_ = string_parameters_[2].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "IK plugin name: %s", ik_plugin_name_.c_str());
      control_frame_ = string_parameters_[3].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "Control frame: %s", control_frame_.c_str());
      sensor_frame_ = string_parameters_[4].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "Sensor frame: %s", sensor_frame_.c_str());
      end_effector_name_ = string_parameters_[5].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "end effector name: %s", sensor_frame_.c_str());
      cog_frame_ = string_parameters_[6].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "COG frame: %s", cog_frame_.c_str());

      open_loop_control_ = bool_parameters_[0].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "Using open loop: %s", (open_loop_control_ ? "True" : "False"));
      enable_parameter_update_without_reactivation_ = bool_parameters_[1].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "Using update without reactivation: %s", (enable_parameter_update_without_reactivation_ ? "True" : "False"));


      int offset_index_bool = 2;  // 2 because there are already two parameters used above
      for (size_t i = 0; i < 6; ++i) {
        selected_axes_[i] = bool_parameters_[i + offset_index_bool].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Axis %zu is %sselected", i, (selected_axes_[i] ? "" : "not "));

        mass_[i] = double_parameters_[i].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Mass for the axis %zu is %e", i, mass_[i]);
        stiffness_[i] = double_parameters_[i + 6].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Stiffness for the axis %zu is %e", i, stiffness_[i]);
        damping_[i] = double_parameters_[i + 12].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Damping for the axis %zu is %e", i, damping_[i]);
        damping_ratio_[i] = double_parameters_[i + 18].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Damping_ratio for the axis %zu is %e", i, damping_ratio_[i]);
      }
      convert_damping_ratio_to_damping();

      for (int i = 0; i < 3; i++){
        cog_[i] = double_parameters_[24 + i].second;
        RCUTILS_LOG_INFO_NAMED(
            logger_name_.c_str(),
            "Damping_ratio for the axis %zu is %e", i, cog_[i]);
      }
      force_ = double_parameters_[27].second;
      RCUTILS_LOG_INFO_NAMED(
          logger_name_.c_str(),
          "CoG force %e", force_);
    }

    // IK parameters
    std::string ik_base_frame_;
    std::string ik_group_name_;
    std::string ik_plugin_name_;
    std::string end_effector_name_;
    // Depends on the scenario: usually base_link, tool or end-effector
    std::string control_frame_;
    // Admittance calculations (displacement etc) are done in this frame.
    // Frame where wrench measurements are taken
    std::string sensor_frame_;
    std::string cog_frame_;

    bool open_loop_control_;
    bool enable_parameter_update_without_reactivation_;

    std::array<double, 6> damping_;
    std::array<double, 6> damping_ratio_;
    std::array<double, 3> cog_;
    std::array<double, 6> mass_;
    std::array<bool, 6> selected_axes_;
    std::array<double, 6> stiffness_;
    double force_;
  };


  class AdmittanceRule {
  public:
    AdmittanceRule() = default;

    controller_interface::return_type configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int num_joint);

    controller_interface::return_type reset();

//  controller_interface::return_type update(
//    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
//    const geometry_msgs::msg::Wrench & measured_wrench,
//    const geometry_msgs::msg::PoseStamped & reference_pose,
//    const rclcpp::Duration & period,
//    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
//  );
//
//  controller_interface::return_type update(
//    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
//    const geometry_msgs::msg::Wrench & measured_wrench,
//    const std::array<double, 6> & reference_joint_deltas,
//    const rclcpp::Duration & period,
//    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
//  );

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

    controller_interface::return_type get_controller_state(
        control_msgs::msg::AdmittanceControllerState &state_message
    );


  public:
    // TODO(destogl): Add parameter for this
    bool feedforward_commanded_input_ = true;


    // Dynamic admittance parameters
    AdmittanceParameters parameters_;

    // Filter parameter for exponential smoothing
    const double alpha = 0.1; // TODO make a ros param
    const double EPSILON = 0.001;


  protected:
    /**
     * All values are in he controller frame
     */
    void calculate_admittance_rule(
        const Eigen::Matrix<double,6,1> &wrench,
        const rclcpp::Duration &period
    );

    void process_wrench_measurements(
        const geometry_msgs::msg::Wrench &measured_wrench
    );

    Eigen::Matrix4d invert_transform(const Eigen::Matrix4d &T);
    Eigen::Vector3d get_rotation_axis(const Eigen::Matrix3d& R) const;
    Eigen::Matrix3d normalize_rotation(Eigen::Matrix3d R);

    // Differential IK algorithm (loads a plugin)
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> ik_loader_;
    std::unique_ptr<kinematics_interface::KinematicsBaseClass> ik_;

    //
    int num_joints_;

    Eigen::Matrix<double,6,1> admittance_acceleration_;
    std::vector<double> admittance_acceleration_vec;
    Eigen::Matrix<double,6,1> admittance_velocity_;
    std::vector<double> admittance_velocity_vec;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> admittance_position_;
    std::vector<double> admittance_position_vec;

    Eigen::Matrix<double,4,4,Eigen::ColMajor> cur_ee_transform;
    std::vector<double> cur_ee_transform_vec;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> desired_ee_transform;
    std::vector<double> desired_ee_transform_vec;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> cur_sensor_transform;
    std::vector<double> cur_sensor_transform_vec;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> cur_control_transform;
    std::vector<double> cur_control_transform_vec;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> cog_transform;
    std::vector<double> cog_transform_vec;

    Eigen::Matrix<double,6,1> wrench;
    Eigen::Matrix<double,6,1> desired_ee_vel;
    Eigen::Matrix<double,3,1> cog_;
    Eigen::Matrix<double,3,1> ee_weight;
    std::vector<double> desired_ee_vel_vec;

    std::vector<double> joint_vel;
    std::vector<double> joint_acc;
    std::vector<double> joint_pos;

    trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;

  private:

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
