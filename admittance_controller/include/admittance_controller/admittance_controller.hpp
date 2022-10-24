// Copyright (c) 2022, PickNik, Inc.
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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "admittance_controller_parameters.hpp"

#include "admittance_controller/admittance_rule.hpp"
#include "admittance_controller/visibility_control.h"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace admittance_controller
{
using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;

class AdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  size_t num_joints_ = 0;
  std::vector<std::string> command_joint_names_;

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;
  bool has_effort_command_interface_ = false;

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION};

  // internal reference values
  const std::vector<std::string> allowed_reference_interfaces_types_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  std::vector<std::reference_wrapper<double>> position_reference_;
  std::vector<std::reference_wrapper<double>> velocity_reference_;

  // Admittance rule and dependent variables;
  std::unique_ptr<admittance_controller::AdmittanceRule> admittance_;

  // force torque sensor
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // ROS subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
    input_joint_command_subscriber_;
  rclcpp::Publisher<control_msgs::msg::AdmittanceControllerState>::SharedPtr s_publisher_;

  // admittance parameters
  std::shared_ptr<admittance_controller::ParamListener> parameter_handler_;

  // ROS messages
  std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> joint_command_msg_;

  // real-time buffer
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>>
    input_joint_command_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateMsg>> state_publisher_;

  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_;
  trajectory_msgs::msg::JointTrajectoryPoint last_reference_;

  // control loop data
  // reference_: reference value read by the controller
  // joint_state_: current joint readings from the hardware
  // reference_admittance_: reference value used by the controller after the admittance values are
  // applied ft_values_: values read from the force torque sensor
  trajectory_msgs::msg::JointTrajectoryPoint reference_, joint_state_, reference_admittance_;
  geometry_msgs::msg::Wrench ft_values_;

  /**
  * @brief Read values from hardware interfaces and set corresponding fields of state_current and ft_values
  */
  void read_state_from_hardware(
    trajectory_msgs::msg::JointTrajectoryPoint & state_current,
    geometry_msgs::msg::Wrench & ft_values);

  /**
  * @brief Set fields of state_reference with values from controllers exported position and velocity references
  */
  void read_state_reference_interfaces(trajectory_msgs::msg::JointTrajectoryPoint & state);

  /**
* @brief Write values from state_command to claimed hardware interfaces
*/
  void write_state_to_hardware(const trajectory_msgs::msg::JointTrajectoryPoint & state_command);
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
