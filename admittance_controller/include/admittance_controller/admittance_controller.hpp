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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule.hpp"
#include "admittance_controller/visibility_control.h"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
// TODO(destogl): this is only temporary to work with servo. It should be either trajectory_msgs/msg/JointTrajectoryPoint or std_msgs/msg/Float64MultiArray
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace admittance_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::string ft_sensor_name_;
  bool use_joint_commands_as_input_;

  bool hardware_state_has_offset_;
  trajectory_msgs::msg::JointTrajectoryPoint current_state_when_offset_;

  // Internal variables
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // Admittance rule and dependent variables;
  std::unique_ptr<admittance_controller::AdmittanceRule> admittance_;
  rclcpp::Time previous_time_;

  // Command subscribers and Controller State publisher
  using ControllerCommandWrenchMsg = geometry_msgs::msg::WrenchStamped;
  using ControllerCommandPoseMsg = geometry_msgs::msg::PoseStamped;
  using ControllerCommandJointMsg = trajectory_msgs::msg::JointTrajectory;

  rclcpp::Subscription<ControllerCommandWrenchMsg>::SharedPtr
  input_wrench_command_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerCommandPoseMsg>::SharedPtr
  input_pose_command_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerCommandJointMsg>::SharedPtr input_joint_command_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandWrenchMsg>>
  input_wrench_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandPoseMsg>>
  input_pose_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandJointMsg>>
  input_joint_command_;

  using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // Internal access to sorted interfaces

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_velocity_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;

};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
