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

#include "admittance_controller/admittance_rule.hpp"
#include "admittance_controller/visibility_control.h"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

// TODO(destogl): this is only temporary to work with servo. It should be either trajectory_msgs/msg/JointTrajectoryPoint or std_msgs/msg/Float64MultiArray
#include "trajectory_msgs/msg/joint_trajectory.hpp"


using namespace std::chrono_literals;

namespace admittance_controller {
  using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class AdmittanceController : public controller_interface::ChainableControllerInterface {
  public:
    ADMITTANCE_CONTROLLER_PUBLIC

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_init() override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;


  protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    bool on_set_chained_mode(bool chained_mode) override;

    size_t num_joints_ = 0;
    size_t state_pos_ind = -1;
    size_t state_vel_ind = -1;
    size_t state_acc_ind = -1;
    size_t command_pos_ind = -1;
    size_t command_vel_ind = -1;
    size_t command_acc_ind = -1;

    // internal reference values
    const std::vector<std::string> reference_interfaces_types_ = {"position", "velocity"};
    std::vector<std::reference_wrapper<double>> position_reference_;
    std::vector<std::reference_wrapper<double>> velocity_reference_;

    // Admittance rule and dependent variables;
    std::unique_ptr<admittance_controller::AdmittanceRule> admittance_;

    // force torque sensor
    std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

    // ROS subscribers
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr input_joint_command_subscriber_;
    rclcpp::Publisher<control_msgs::msg::AdmittanceControllerState>::SharedPtr s_publisher_;

    // ROS messages
    std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> joint_command_msg;

    // real-time buffer
    realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>> input_joint_command_;
    std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateMsg>> state_publisher_;

    trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
    trajectory_msgs::msg::JointTrajectoryPoint last_state_reference_;
    trajectory_msgs::msg::JointTrajectoryPoint state_offset_;
    trajectory_msgs::msg::JointTrajectoryPoint prev_trajectory_point_;

    // control loop data
    trajectory_msgs::msg::JointTrajectoryPoint state_reference_, state_current_, state_desired_, state_error_;
    geometry_msgs::msg::Wrench ft_values_;
    trajectory_msgs::msg::JointTrajectory pre_admittance_point;
    size_t loop_counter = 0;

    // helper methods
    void read_state_from_hardware(trajectory_msgs::msg::JointTrajectoryPoint &state_current,
                                  geometry_msgs::msg::Wrench &ft_values);
    void read_state_reference_interfaces(trajectory_msgs::msg::JointTrajectoryPoint &state);
    void write_state_to_hardware(const trajectory_msgs::msg::JointTrajectoryPoint &state_commanded);

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
