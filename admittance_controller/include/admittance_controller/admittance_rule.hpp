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

#include <map>

#include "angles/angles.h"
#include "admittance_controller/moveit_kinematics.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_parameters.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rcutils/logging_macros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>

namespace {  // Utility namespace

// Numerical accuracy checks. Used as deadbands.
static constexpr double WRENCH_EPSILON = 1e-10;
static constexpr double POSE_ERROR_EPSILON = 1e-12;
static constexpr double POSE_EPSILON = 1e-15;

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Pose & msg, Type & vector_out)
{
  vector_out[0] = msg.position.x;
  vector_out[1] = msg.position.y;
  vector_out[2] = msg.position.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::PoseStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.pose, vector_out);
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Transform & msg, Type & vector_out)
{
  vector_out[0] = msg.translation.x;
  vector_out[1] = msg.translation.y;
  vector_out[2] = msg.translation.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.rotation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::TransformStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.transform, vector_out);
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::Wrench & msg, Type & vector_out)
{
  vector_out[0] = msg.force.x;
  vector_out[1] = msg.force.y;
  vector_out[2] = msg.force.z;
  vector_out[3] = msg.torque.x;
  vector_out[4] = msg.torque.y;
  vector_out[5] = msg.torque.z;
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::WrenchStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.wrench, vector_out);
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Pose & msg_out)
{
  msg_out.position.x = vector[0];
  msg_out.position.y = vector[1];
  msg_out.position.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.orientation.x = q.x();
  msg_out.orientation.y = q.y();
  msg_out.orientation.z = q.z();
  msg_out.orientation.w = q.w();
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::PoseStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.pose);
}

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::Wrench & msg_out)
// {
//   msg_out.force.x = vector[0];
//   msg_out.force.y = vector[1];
//   msg_out.force.z = vector[2];
//   msg_out.torque.x = vector[3];
//   msg_out.torque.y = vector[4];
//   msg_out.torque.z = vector[5];
// }

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::WrenchStamped & msg_out)
// {
//   convert_array_to_message(vector, msg_out.wrench);
// }

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Transform & msg_out)
{
  msg_out.translation.x = vector[0];
  msg_out.translation.y = vector[1];
  msg_out.translation.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.rotation.x = q.x();
  msg_out.rotation.y = q.y();
  msg_out.rotation.z = q.z();
  msg_out.rotation.w = q.w();
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::TransformStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.transform);
}

}  // utility namespace

namespace admittance_controller
{

class AdmittanceParameters : public controller_interface::ControllerParameters
{
public:
  AdmittanceParameters() : controller_interface::ControllerParameters(7, 24, 4)
  {
    add_string_parameter("IK.base", false);
    add_string_parameter("IK.group_name", false);
    add_string_parameter("control_frame", true);
    add_string_parameter("sensor_frame", false);

    add_bool_parameter("open_loop_control", true);

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
  }

  bool check_if_parameters_are_valid() override
  {
    bool ret = true;

    // Check if any string parameter is empty
    ret = !empty_parameter_in_list(string_parameters_);

    int index = 0;
    int offset_index_bool = 1;
    // check if parameters are all properly set for selected axes
    for (auto i = 0ul; i < 6; ++i) {
      if (bool_parameters_[offset_index_bool + i].second)
      {
        // check mass parameters
        index = i;
        if (std::isnan(double_parameters_[index].second))
        {
          RCUTILS_LOG_ERROR_NAMED(
            logger_name_.c_str(),
            "Parameter '%s' has to be set", double_parameters_[index].first.name.c_str());
          ret = false;
        }
        // Check stiffness parameters
        index = i + 6;
        if (std::isnan(double_parameters_[index].second))
        {
          RCUTILS_LOG_ERROR_NAMED(
            logger_name_.c_str(),
            "Parameter '%s' has to be set", double_parameters_[index].first.name.c_str());
          ret = false;
        }
        // Check damping or damping_ratio parameters
        index = i + 12;
        if (std::isnan(double_parameters_[index].second) &&
            std::isnan(double_parameters_[index + 6].second))
        {
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
  void convert_damping_ratio_to_damping()
  {
    for (auto i = 0ul; i < damping_ratio_.size(); ++i)
    {
      if (!std::isnan( damping_ratio_[i]))
      {
        damping_[i] = damping_ratio_[i] * 2 * sqrt(mass_[i] * stiffness_[i]);
      }
      else
      {
        RCUTILS_LOG_DEBUG_NAMED(
          logger_name_.c_str(),
          "Damping ratio for axis %zu not used because it is NaN.", i);
      }
    }
  }

  void update() override
  {
    ik_base_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "IK Base frame: %s", ik_base_frame_.c_str());
    ik_group_name_ = string_parameters_[1].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "IK group name frame: %s", ik_group_name_.c_str());
    control_frame_ = string_parameters_[2].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Control frame: %s", control_frame_.c_str());
    sensor_frame_ = string_parameters_[3].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Sensor frame: %s", sensor_frame_.c_str());

    open_loop_control_ = bool_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Using open loop: %s", (open_loop_control_ ? "True" : "False"));

    for (auto i = 0ul; i < 6; ++i)
    {
      selected_axes_[i] = bool_parameters_[i+1].second; // +1 because there is already one parameter
      RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Axis %zu is %sselected", i, (selected_axes_[i] ? "" : "not "));

      mass_[i] = double_parameters_[i].second;
      RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Mass for the axis %zu is %e", i, mass_[i]);
      stiffness_[i] = double_parameters_[i+6].second;
      RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Stiffness for the axis %zu is %e", i, stiffness_[i]);
      damping_[i] = double_parameters_[i+12].second;
      RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
        "Damping for the axis %zu is %e", i, damping_[i]);
      damping_ratio_[i] = double_parameters_[i+18].second;
      RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Damping_ratio for the axis %zu is %e", i, damping_ratio_[i]);
    }

    convert_damping_ratio_to_damping();
  }

  // IK parameters
  std::string ik_base_frame_;
  std::string ik_group_name_;
  // Admittance calculations (displacement etc) are done in this frame.
  // Frame where wrench measurements are taken
  std::string sensor_frame_;
  // Depends on the scenario: usually base_link, tool or end-effector
  std::string control_frame_;

  bool open_loop_control_;

  std::array<double, 6> damping_;
  std::array<double, 6> damping_ratio_;
  std::array<double, 6> mass_;
  std::array<bool, 6> selected_axes_;
  std::array<double, 6> stiffness_;
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
  // TODO(destogl): Add parameter for this
  bool feedforward_commanded_input_ = true;

  // An identity matrix is needed in several places
  geometry_msgs::msg::TransformStamped identity_transform_;

  // Admittance parameters
  // TODO(destogl): unified mode does not have to be here
  bool unified_mode_ = false;  // Unified mode enables simultaneous force and position goals

  // Dynamic admittance parameters
  AdmittanceParameters parameters_;

  // Filter chain for Wrench data
  std::unique_ptr<filters::FilterChain<geometry_msgs::msg::WrenchStamped>> filter_chain_;

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
    const std::array<double, 6> & relative_pose,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
  );

  // IK variables
  std::shared_ptr<MoveItKinematics> ik_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Transformation variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // measured_wrench_ could arrive in any frame. It will be transformed
  geometry_msgs::msg::WrenchStamped measured_wrench_;
  geometry_msgs::msg::WrenchStamped measured_wrench_filtered_;

  geometry_msgs::msg::WrenchStamped measured_wrench_ik_base_frame_;

  geometry_msgs::msg::PoseStamped current_pose_ik_base_frame_;
  geometry_msgs::msg::PoseStamped current_pose_control_frame_;

  // This is the feedforward pose. Where should the end effector be with no wrench applied?
  geometry_msgs::msg::PoseStamped reference_pose_from_joint_deltas_ik_base_frame_;
  std::array<double, 6> feedforward_velocity_ik_base_frame_;
  // Need to save the previous velocity to calculate acceleration
  std::array<double, 6> prev_feedforward_velocity_ik_base_frame_;

  geometry_msgs::msg::WrenchStamped reference_force_ik_base_frame_;
  geometry_msgs::msg::PoseStamped reference_pose_ik_base_frame_;
  geometry_msgs::msg::PoseStamped reference_pose_control_frame_;

  geometry_msgs::msg::PoseStamped admittance_pose_ik_base_frame_;
  geometry_msgs::msg::PoseStamped sum_of_admittance_displacements_;
  geometry_msgs::msg::PoseStamped sum_of_admittance_displacements_control_frame_;
  geometry_msgs::msg::TransformStamped relative_admittance_pose_ik_base_frame_;
  geometry_msgs::msg::TransformStamped relative_admittance_pose_control_frame_;
  geometry_msgs::msg::TransformStamped admittance_velocity_ik_base_frame_;
  geometry_msgs::msg::TransformStamped admittance_velocity_control_frame_;

  // Joint deltas calculation variables
  std::vector<double> reference_joint_deltas_vec_;
  std::vector<double> reference_deltas_vec_ik_base_;
  geometry_msgs::msg::TransformStamped reference_deltas_ik_base_;

  bool movement_caused_by_wrench_ = false;

  // Pre-reserved update-loop variables
  std::array<double, 6> measured_wrench_ik_base_frame_arr_;
  std::array<double, 6> reference_pose_arr_;
  std::array<double, 6> current_pose_arr_;

  std::array<double, 6> relative_admittance_pose_arr_;
  std::array<double, 6> admittance_pose_ik_base_frame_arr_;
  std::array<double, 6> admittance_velocity_arr_;
  // Keep a running tally of motion due to admittance, to calculate spring force in open-loop mode
  std::array<double, 6> sum_of_admittance_displacements_arr_;

  std::vector<double> relative_desired_joint_state_vec_;

  // TODO(destogl): find out better datatype for this
  // Values calculated by admittance rule (Cartesian space: [x, y, z, rx, ry, rz]) - state output
  // "positions" hold "pose_error" values
  // "effort" hold "measured_wrench" values
  trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;

private:
  template<typename MsgType>
  controller_interface::return_type
  transform_to_control_frame(const MsgType & message_in, MsgType & message_out)
  {
    return transform_to_frame(message_in, message_out, parameters_.control_frame_);
  }

  template<typename MsgType>
  controller_interface::return_type
  transform_to_ik_base_frame(const MsgType & message_in, MsgType & message_out)
  {
    return transform_to_frame(message_in, message_out, parameters_.ik_base_frame_);
  }

  template<typename MsgType>
  controller_interface::return_type
  transform_to_frame(const MsgType & message_in, MsgType & message_out, const std::string & frame)
  {
    if (frame != message_in.header.frame_id) {
      try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          frame, message_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(message_in, message_out, transform);
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(
          rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
        return controller_interface::return_type::ERROR;
      }
    } else {
      message_out = message_in;
    }
    return controller_interface::return_type::OK;
  }

  template<typename MsgType>
  controller_interface::return_type
  transform_relative_to_control_frame(const MsgType & message_in, MsgType & message_out)
  {
    return transform_relative_to_frame(message_in, message_out, parameters_.control_frame_);
  }

  template<typename MsgType>
  controller_interface::return_type
  transform_relative_to_ik_base_frame(const MsgType & message_in, MsgType & message_out)
  {
    return transform_relative_to_frame(message_in, message_out, parameters_.ik_base_frame_);
  }

  /**
   * Transforms relative movement/pose to a new frame.
   * Consider following discussion/approaches in the future the:
   *   - https://answers.ros.org/question/192273/how-to-implement-velocity-transformation/?answer=192283#post-id-192283
   *   - https://physics.stackexchange.com/questions/197009/transform-velocities-from-one-frame-to-an-other-within-a-rigid-body#244364
   */
  template<typename MsgType>
  controller_interface::return_type
  transform_relative_to_frame(const MsgType & message_in, MsgType & message_out, const std::string & frame)
  {
    controller_interface::return_type ret = controller_interface::return_type::OK;

    // Do calculation only if transformation needed
    if (frame != message_in.header.frame_id) {

      MsgType message_transformed;
      std::array<double, 6> message_transformed_arr;

      geometry_msgs::msg::PoseStamped zero_pose;
      geometry_msgs::msg::PoseStamped zero_pose_transformed;
      std::array<double, 6> zero_pose_transformed_arr;

      std::array<double, 6> relative_message_transformed_arr;

      ret = transform_to_frame(message_in, message_transformed, frame);
      convert_message_to_array(message_transformed, message_transformed_arr);

      zero_pose.header.frame_id = message_in.header.frame_id;
      ret = transform_to_frame(zero_pose, zero_pose_transformed, frame);
      convert_message_to_array(zero_pose_transformed, zero_pose_transformed_arr);

      for (auto i = 0u; i < 6; ++i) {
        relative_message_transformed_arr[i] =
          message_transformed_arr[i] - zero_pose_transformed_arr[i];

        if (i >= 3) {
          relative_message_transformed_arr[i] =
            angles::normalize_angle(relative_message_transformed_arr[i]);
        }

        if (std::fabs(relative_message_transformed_arr[i]) < POSE_ERROR_EPSILON) {
          relative_message_transformed_arr[i] = 0.0;
        }
      }
      message_out.header = message_transformed.header;
      convert_array_to_message(relative_message_transformed_arr, message_out);
    } else {
      message_out = message_in;
    }

    return ret;
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
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
