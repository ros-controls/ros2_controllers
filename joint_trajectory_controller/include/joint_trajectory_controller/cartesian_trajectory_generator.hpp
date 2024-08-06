// Copyright (c) 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <vector>

#include "control_msgs/msg/cartesian_trajectory_generator_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace cartesian_trajectory_generator
{
class CartesianTrajectoryGenerator : public joint_trajectory_controller::JointTrajectoryController
{
public:
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  CartesianTrajectoryGenerator();

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  using ControllerReferenceMsg = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint;
  using ControllerFeedbackMsg = nav_msgs::msg::Odometry;

protected:
  bool read_state_from_hardware(JointTrajectoryPoint & state) override;

  using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void publish_state(
    const rclcpp::Time & time, const JointTrajectoryPoint & desired_state,
    const JointTrajectoryPoint & current_state, const JointTrajectoryPoint & state_error,
    const JointTrajectoryPoint & splines_output, const JointTrajectoryPoint & ruckig_input_target,
    const JointTrajectoryPoint & ruckig_input) override;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_reliable_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> reference_world_;

  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> feedback_;

  trajectory_msgs::msg::JointTrajectoryPoint control_output_local_;

private:
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  using CartControllerStateMsg = control_msgs::msg::CartesianTrajectoryGeneratorState;
  using CartStatePublisher = realtime_tools::RealtimePublisher<CartControllerStateMsg>;
  using CartStatePublisherPtr = std::unique_ptr<CartStatePublisher>;
  rclcpp::Publisher<CartControllerStateMsg>::SharedPtr cart_publisher_;
  CartStatePublisherPtr cart_state_publisher_;

  std::vector<joint_limits::JointLimits> configured_joint_limits_;

  // storage of last received measured position to
  geometry_msgs::msg::Pose last_received_measured_position_;
};

}  // namespace cartesian_trajectory_generator

#endif  // JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
