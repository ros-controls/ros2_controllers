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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include "admittance_controller/moveit_kinematics.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
// TODO(destogl): Enable use of filters
// #include "iirob_filters/gravity_compensation.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace admittance_controller
{
struct GravityCompensationParameters
{
public:
  std::string world_frame_;
  std::string sensor_frame_;
  geometry_msgs::msg::Vector3Stamped cog_;
  double force_;
};

class AdmittanceRule
{
public:
  AdmittanceRule() = default;

  controller_interface::return_type configure(rclcpp::Node::SharedPtr node);

  controller_interface::return_type reset();

  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const geometry_msgs::msg::PoseStamped & reference_pose,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const std::array<double, 6> & reference_joint_deltas,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const geometry_msgs::msg::PoseStamped & reference_pose,
    const geometry_msgs::msg::WrenchStamped & reference_force,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type get_controller_state(
    control_msgs::msg::AdmittanceControllerState & state_message
  );

  controller_interface::return_type get_pose_of_control_frame_in_base_frame(geometry_msgs::msg::PoseStamped & pose);

public:
  bool open_loop_control_ = false;
  // TODO(destogl): Add parameter for this
  bool feedforward_commanded_input_ = true;

  // IK related parameters
  // ik_base_frame should be stationary so vel/accel calculations are correct
  std::string ik_base_frame_;
  std::string ik_tip_frame_;
  std::string ik_group_name_;

  // Frame which position should be controlled
  std::string endeffector_frame_;
  // Admittance calcs (displacement etc) are done in this frame. Usually the tool or end-effector
  std::string control_frame_;
  // Gravity points down (neg. Z) in the world frame
  std::string fixed_world_frame_;
  // Frame where wrench measurements are taken
  std::string sensor_frame_;

  // An identity matrix is needed in several places
  geometry_msgs::msg::TransformStamped identity_transform_;

  // Admittance parameters
  // TODO(destogl): unified mode does not have to be here
  bool unified_mode_ = false;  // Unified mode enables simultaneous force and position goals
  std::array<bool, 6> selected_axes_;
  std::array<double, 6> mass_;
  std::array<double, 6> damping_;
  std::array<double, 6> stiffness_;

  // Filters
  std::vector<GravityCompensationParameters> gravity_compensation_params_;

protected:
  void process_wrench_measurements(
    const geometry_msgs::msg::Wrench & measured_wrench
  );

  /**
   * All values are in he controller frame
   */
  void calculate_admittance_rule(
    const std::array<double, 6> & measured_wrench,
    const std::array<double, 6> & pose_error,
    const rclcpp::Duration & period,
    std::array<double, 6> & desired_relative_pose
  );

  controller_interface::return_type calculate_desired_joint_state(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
  );

  // IK variables
  std::shared_ptr<MoveItKinematics> ik_;

  // Filters
//   using GravityCompensatorType =
//     iirob_filters::GravityCompensator<geometry_msgs::msg::WrenchStamped>;
//
//   std::unique_ptr<GravityCompensatorType> wrist_gravity_compensator_;
//   std::unique_ptr<GravityCompensatorType> tool_gravity_compensator_;

  // Transformation variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  tf2::Transform ik_tip_to_control_frame_tf_;
  tf2::Transform control_frame_to_ik_tip_tf_;

  // measured_wrench_ could arrive in any frame. It will be transformed
  geometry_msgs::msg::WrenchStamped measured_wrench_;
  geometry_msgs::msg::WrenchStamped measured_wrench_filtered_;

  geometry_msgs::msg::WrenchStamped measured_wrench_ik_base_frame_;
  geometry_msgs::msg::WrenchStamped measured_wrench_endeffector_frame_;

  geometry_msgs::msg::PoseStamped current_pose_ik_base_frame_;

  // This is the feedforward pose. Where should the end effector be with no wrench applied?
  geometry_msgs::msg::PoseStamped reference_pose_from_joint_deltas_ik_base_frame_;
  std::array<double, 6> feedforward_velocity_ik_base_frame_;
  // Need to save the previous velocity to calculate acceleration
  std::array<double, 6> prev_feedforward_velocity_ik_base_frame_;

  geometry_msgs::msg::WrenchStamped reference_force_ik_base_frame_;
  geometry_msgs::msg::PoseStamped reference_pose_ik_base_frame_;

  geometry_msgs::msg::PoseStamped desired_pose_ik_base_frame_;
  geometry_msgs::msg::TransformStamped relative_desired_pose_;

  bool movement_caused_by_wrench_ = false;

  // Pre-reserved update-loop variables
  std::array<double, 6> measured_wrench_ik_base_frame_arr_;
  std::array<double, 6> reference_pose_ik_base_frame_arr_;
  std::array<double, 6> current_pose_ik_base_frame_arr_;

  std::array<double, 6> relative_desired_pose_arr_;
  std::array<double, 6> desired_pose_ik_base_frame_arr_;
  std::array<double, 6> admittance_velocity_arr_;

  std::vector<double> relative_desired_joint_state_vec_;

  // TODO(destogl): find out better datatype for this
  // Values calculated by admittance rule (Cartesian space: [x, y, z, rx, ry, rz]) - state output
  // "positions" hold "pose_error" values
  // "effort" hold "measured_wrench" values
  trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;

private:
  template<typename MsgType>
  controller_interface::return_type
  transform_message_to_ik_base_frame(const MsgType & message_in, MsgType & message_out)
  {
    if (ik_base_frame_ != message_in.header.frame_id) {
      try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          ik_base_frame_, message_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(message_in, message_out, transform);
      } catch (const tf2::TransformException & e) {
        // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
        message_in.header.frame_id + "' to '" + ik_base_frame_ + "'.");
        return controller_interface::return_type::ERROR;
      }
    } else {
      message_out = message_in;
    }
    return controller_interface::return_type::OK;
  }

  template<typename Type>
  void
  direct_transform(const Type & input, const tf2::Transform & transform, Type & output)
  {
    // use TF2 data types for easy math
    tf2::Transform input_tf, output_tf;

    tf2::fromMsg(input, input_tf);
    output_tf = input_tf * transform;
    tf2::toMsg(output_tf, output);
  }

  // TODO(destogl): As of C++17 use transform_reduce:
  // https://stackoverflow.com/questions/58266717/accumulate-absolute-values-of-a-vector
  template<typename Type>
  double accumulate_absolute(const Type & container) {
    double accumulator = 0.0;
    for (auto i = 0ul; i < container.size(); i++) {
      accumulator += std::fabs(container[i]);
    }
    return accumulator;
  }
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
