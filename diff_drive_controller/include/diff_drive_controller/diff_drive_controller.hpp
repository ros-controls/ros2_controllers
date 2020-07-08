// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"
#include "diff_drive_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/joint_command_handle.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace diff_drive_controller
{
class DiffDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  DIFF_DRIVE_CONTROLLER_PUBLIC
  DiffDriveController();

  DIFF_DRIVE_CONTROLLER_PUBLIC
  DiffDriveController(
    std::vector<std::string> left_wheel_names,
    std::vector<std::string> right_wheel_names,
    std::vector<std::string> operation_mode_names);

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type
  init(
    std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
    const std::string & controller_name) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    const hardware_interface::JointStateHandle * state = nullptr;
    hardware_interface::JointCommandHandle * command = nullptr;
  };

  CallbackReturn configure_side(
    const std::string & side,
    const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles,
    hardware_interface::RobotHardware & robot_hardware);

  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  struct WheelParams
  {
    size_t wheels_per_side = 0;
    double separation = 0.0;   // w.r.t. the midpoint of the wheel width
    double radius = 0.0;       // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odometry_publisher_
    =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>>
  odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
  realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  std::vector<std::string> write_op_names_;
  std::vector<hardware_interface::OperationModeHandle *> registered_operation_mode_handles_;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

  std::shared_ptr<Twist> received_velocity_msg_ptr_ = nullptr;

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<Twist>> limited_velocity_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>>
  realtime_limited_velocity_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  bool is_halted = false;

  bool reset();
  void set_op_mode(const hardware_interface::OperationMode & mode);
  void halt();
};
}  // namespace diff_drive_controller
#endif  // DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
