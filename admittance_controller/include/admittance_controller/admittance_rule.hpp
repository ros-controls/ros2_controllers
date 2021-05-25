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
/// \author: Denis Stogl

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include "admittance_controller/incremental_kinematics.hpp"
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
    const geometry_msgs::msg::PoseStamped & target_pose,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const std::array<double, 6> & target_joint_deltas,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const geometry_msgs::msg::WrenchStamped & target_force,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states
  );

  controller_interface::return_type get_controller_state(
    control_msgs::msg::AdmittanceControllerState & state_message
  );

  controller_interface::return_type get_pose_of_control_frame_in_base_frame(geometry_msgs::msg::PoseStamped & pose);

public:
  bool hardware_state_has_offset_ = false;

  // IK related parameters
  std::string ik_base_frame_;
  std::string ik_tip_frame_;
  std::string ik_group_name_;

  // Controller frames
  std::string control_frame_;
  std::string fixed_world_frame_;
  std::string sensor_frame_;

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
  void process_force_measurements(
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
  std::shared_ptr<IncrementalKinematics> ik_;

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

  geometry_msgs::msg::WrenchStamped measured_wrench_control_frame_;
  geometry_msgs::msg::WrenchStamped measured_wrench_endeffector_frame_;

  geometry_msgs::msg::PoseStamped origin_ik_tip_;
  geometry_msgs::msg::PoseStamped origin_endeffector_;
  geometry_msgs::msg::PoseStamped current_pose_base_frame_;
  geometry_msgs::msg::PoseStamped current_pose_control_frame_;

  geometry_msgs::msg::WrenchStamped target_force_control_frame_;
  geometry_msgs::msg::PoseStamped target_pose_control_frame_;

  geometry_msgs::msg::PoseStamped desired_pose_control_frame_;
  geometry_msgs::msg::TransformStamped relative_desired_pose_;

  // Pre-reserved update-loop variables
  std::array<double, 6> measured_wrench_control_frame_arr_;
  std::array<double, 6> target_pose_control_frame_arr_;
  std::array<double, 6> current_pose_control_frame_arr_;

  std::array<double, 3> angles_error_;

  std::array<double, 6> relative_desired_pose_arr_;
  std::array<double, 6> desired_velocity_arr_;
  std::array<double, 6> desired_velocity_previous_arr_;
  std::array<double, 6> desired_acceleration_previous_arr_;

  std::vector<double> relative_desired_joint_state_vec_;

private:
  template<typename MsgType>
  controller_interface::return_type
  transform_message_to_control_frame(const MsgType & message_in, MsgType & message_out)
  {
    if (control_frame_ != message_in.header.frame_id) {
      try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          message_in.header.frame_id, control_frame_, tf2::TimePointZero);
        tf2::doTransform(message_in, message_out, transform);
      } catch (const tf2::TransformException & e) {
        // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + control_frame_ + "' and '" + message_in.header.frame_id + "'.");
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

  template<typename Type>
  void
  transform_ik_tip_to_control_frame(const Type & base_to_ik_tip, Type & base_to_toollink)
  {
    direct_transform(base_to_ik_tip, ik_tip_to_control_frame_tf_, base_to_toollink);
  }

  template<typename Type>
  void
  transform_control_to_ik_tip_frame(const Type & base_to_toollink, Type & base_to_ik_tip)
  {
    direct_transform(base_to_toollink, control_frame_to_ik_tip_tf_, base_to_ik_tip);
  }

};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
