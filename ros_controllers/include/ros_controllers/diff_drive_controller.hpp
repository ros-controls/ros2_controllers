// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_
#define ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_controllers/visibility_control.h"
#include "sensor_msgs/msg/joint_state.hpp"

using JointHandle = std::unique_ptr<hardware_interface::JointCommandHandle>;
using Twist = geometry_msgs::msg::Twist;
using JointState = sensor_msgs::msg::JointState;

namespace ros_controllers
{

// TODO(piraka9011):
//   - Configure parameters from server
//   - Register wheel joints + params
//   - Cleanup/Reset gracefully
//   - Pluginlib
//   - Operation mode
//   - Mutexes
class ROS_CONTROLLERS_PUBLIC DiffDriveController : public controller_interface::ControllerInterface
{
public:
  DiffDriveController() = default;

  ~DiffDriveController() = default;

  controller_interface::controller_interface_ret_t init(
    std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
    const std::string & controller_name) override;

  controller_interface::controller_interface_ret_t update() override;

private:
  std::string name_{};

  rclcpp::Subscription<Twist>::SharedPtr command_subscriber_{nullptr};
  rclcpp::Publisher<JointState> joint_state_publisher_;

  /// Wheel parameters
  std::vector<JointHandle> left_wheel_joints_{};
  std::vector<JointHandle> right_wheel_joints_{};
  size_t wheel_joints_size_;
  double wheel_separation_;   // w.r.t. the midpoint of the wheel width
  double wheel_radius_;       // Assumed to be the same for both wheels
  double wheel_separation_multiplier_;
  double left_wheel_radius_multiplier_;
  double right_wheel_radius_multiplier_;

  /// Commands
  // Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;
  std::vector<double> vel_left_previous_{};
  std::vector<double> vel_right_previous_{};

  /// Odom
  // Frame to use for the robot base:
  std::string base_frame_id_;
  // Frame to use for odometry and odom tf:
  std::string odom_frame_id_;

};
}
#endif  // ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_
