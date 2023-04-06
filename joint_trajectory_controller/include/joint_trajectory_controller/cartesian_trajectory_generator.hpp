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
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/srv/set_dof_limits.hpp"
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
  using SetLimitsModeSrvType = control_msgs::srv::SetDOFLimits;

protected:
  void read_state_from_hardware(JointTrajectoryPoint & state) override;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_reliable_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> feedback_;

  rclcpp::Service<SetLimitsModeSrvType>::SharedPtr set_joint_limits_service_;

private:
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  void set_joint_limits_service_callback(
    const std::shared_ptr<SetLimitsModeSrvType::Request> request,
    std::shared_ptr<SetLimitsModeSrvType::Response> response);

  std::vector<joint_limits::JointLimits> configured_joint_limits_;
};

}  // namespace cartesian_trajectory_generator

#endif  // JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
