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

#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
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
    std::vector<std::string> left_wheel_names, std::vector<std::string> right_wheel_names,
    std::vector<std::string> operation_mode_names);

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t init(
    std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
    const std::string & controller_name) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t update() override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    const hardware_interface::JointStateHandle * state = nullptr;
    hardware_interface::JointCommandHandle * command = nullptr;
  };

  struct Commands
  {
    double lin;
    double ang;
    rclcpp::Time stamp;

    Commands()
    : lin(0.0), ang(0.0), stamp{0L, rcl_clock_type_t::RCL_ROS_TIME} {}
  };

  /**
   * \brief Resets all Node resources.
   * \return True if successful.
   */
  bool reset();

  /**
   * \brief Sends a break command to robot
   */
  void halt();

  /**
   * \brief Sets Op mode of the robot hardware
   * \param mode
   */
  void set_op_mode(const hardware_interface::OperationMode & mode);

  CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles,
    hardware_interface::RobotHardware & robot_hardware);

  /**
   * \brief Set wheel configuration using urdf passed via ROS parameter
   * \param left_wheel_name Name of left wheel joint
   * \param right_wheel_name Name of right wheel joint
   * \param separation Wheel separation ref
   * \param radius Wheel radius ref
   * \return true if lookup was successful
   */
  bool set_wheel_params_from_urdf(
    const std::string & left_wheel_name, const std::string & right_wheel_name, double & separation,
    double & radius);

  /**
   * \brief Velocity command subscription callback
   * \param msg
   */
  void on_cmd_vel(Twist::ConstSharedPtr msg);
  /**
   * \brief Publish odom message
   * \param current_time Update cycle time
   * \param orientation
   */
  void publish_odom(const rclcpp::Time & current_time, const tf2::Quaternion & orientation) const;
  /**
   * \brief Publish odom tf message
   * \param current_time Update cycle time
   * \param orientation
   */
  void publish_odom_tf(
    const rclcpp::Time & current_time, const tf2::Quaternion & orientation) const;
  /**
   * \brief Publish limit-applied command velocity message
   * \param current_time Update cycle time
   * \param curr_cmd Command of current update cycle
   */
  void publish_cmd_vel_out(const rclcpp::Time & current_time, const Commands & curr_cmd) const;

  /**
   * \brief Publish wheels' JointTrajectoryControllerState message
   * \param time Update cycle time
   * \param period Time elapse since last update
   * \param curr_cmd Reference to current command
   */
  void publish_wheel_joint_controller_state(
    const rclcpp::Time & time, const rclcpp::Duration & period, const Commands & curr_cmd);
  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  // Adjusted wheel separation (raw_separation * separation_multiplier)
  double wheel_separation_ = 0.0;
  // Number of wheels on each side
  size_t wheels_per_side_ = 0;
  // Adjusted left wheel radius (raw_radius * left_multiplier)
  double left_wheel_radius_ = 0.0;
  // Adjusted right wheel radius (raw_radius * right_multiplier)
  double right_wheel_radius_ = 0.0;

  std::string base_frame_id_ = "base_link";
  std::string odom_frame_id_ = "odom";

  // Odometry pose covariance
  std::array<double, 6> pose_covariance_diagonal_;
  // Odometry twist covariance
  std::array<double, 6> twist_covariance_diagonal_;

  std::vector<std::string> write_op_names_;
  std::vector<hardware_interface::OperationModeHandle *> registered_operation_mode_handles_;

  Odometry odometry_;

  // Timestamp of the last update() call processed
  rclcpp::Time previous_update_timestamp_{0};

  // Flag indicating halt command has been called
  bool is_halted = false;

  // Synchronized velocity command data
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  Commands last0_cmd_;  // Previous command
  Commands last1_cmd_;  // Previous previous command

  // Speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  /// Previous actual velocities from the encoders:
  std::vector<double> vel_left_actual_previous_;
  std::vector<double> vel_right_actual_previous_;
  /// Previous desired velocities
  double vel_left_desired_previous_;
  double vel_right_desired_previous_;

  // Flag for using open loop calculation for odometry
  bool open_loop_ = false;
  // Flag for publishing odom tf message
  bool enable_odom_tf_ = true;
  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};
  // Flag for allowing multiple velocity command publishers
  bool allow_multiple_cmd_vel_publishers_ = true;
  // Flag for publishing limited velocity command
  bool publish_limited_velocity_ = false;
  // Flag for publishing wheels' JointTrajectoryControllerState
  bool publish_wheel_joint_controller_state_ = false;
  // Flag to disable subscriber when lifecycle node is not active
  bool subscriber_is_active_ = false;

  // Velocity command subscriber
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<Twist>> limited_velocity_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  // Wheel joint controller state publisher
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointTrajectoryControllerState>>
  wheel_joint_controller_state_publisher_ = nullptr;
  std::shared_ptr<
    realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>
  realtime_wheel_joint_controller_state_publisher_ = nullptr;

  // Odom publisher
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>>
  odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  realtime_odometry_publisher_ = nullptr;

  // Odom tf publisher
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>>
  odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
  realtime_odometry_transform_publisher_ = nullptr;
};
}  // namespace diff_drive_controller
#endif  // DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
