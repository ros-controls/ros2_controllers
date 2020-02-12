/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar, Enrique Fernández
 * Author: Brighten Lee
 */

#ifndef DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/joint_command_handle.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "controller_interface/controller_interface.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diff_drive_controller/visibility_control.h"
#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"

namespace diff_drive_controller
{

class DiffDriveController : public controller_interface::ControllerInterface
{
public:
  DIFF_DRIVE_CONTROLLER_PUBLIC
  DiffDriveController();

  DIFF_DRIVE_CONTROLLER_PUBLIC
  DiffDriveController(const std::vector<std::string> &left_joint_names,
                      const std::vector<std::string> &right_joint_names);

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t
  init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
       const std::string &controller_name) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t
  update() override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  DIFF_DRIVE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  bool reset();

  // Hardware handles
  std::vector<const hardware_interface::JointStateHandle *>
      registered_left_joint_state_handles_, registered_right_joint_state_handles_;
  std::vector<hardware_interface::JointCommandHandle *>
      registered_left_joint_cmd_handles_, registered_right_joint_cmd_handles_;
  //std::vector<hardware_interface::OperationModeHandle *> registered_operation_mode_handles_;

  // Wheel parameters
  std::vector<std::string> left_joint_names_, right_joint_names_;
  //std::vector<std::string> write_op_names_;
  size_t wheel_joints_size_;             // Number of wheel joints
  double wheel_separation_;              // wrt the midpoint of the wheel width
  double wheel_radius_;                  // assuming it's the same for the left and right wheels
  double wheel_separation_multiplier_;   // Wheel separation calibration multiplier
  double left_wheel_radius_multiplier_;  // Left wheel radius calibration multiplier
  double right_wheel_radius_multiplier_; // Right wheel wheel radius calibration multiplier

  // Odometry
  bool open_loop_;
  bool enable_odom_tf_;       // Whether to publish odometry to tf or not
  std::string base_frame_id_; // Frame to use for the robot base
  std::string odom_frame_id_; // Frame to use for odometry and odom tf
  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;

  std::shared_ptr<Odometry> odometry_ = nullptr;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> pub_odom_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>> pub_tf_odom_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rp_odom_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rp_tf_odom_;

  // Velocity command related
  struct Commands
  {
    double lin;
    double ang;
    rclcpp::Time stamp;

    Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
  };
  Commands command_struct_;
  bool subscriber_is_active_;

  // Whether to allow multiple publishers on cmd_vel topic or not
  bool allow_multiple_cmd_vel_publishers_;

  int velocity_rolling_window_size_;

  // Timeout to consider cmd_vel commands old
  double cmd_vel_timeout_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
  realtime_tools::RealtimeBuffer<Commands> command_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> pub_cmd_vel_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rp_cmd_vel_;

  // Speed limiters
  Commands last1_cmd_;
  Commands last0_cmd_;
  std::shared_ptr<SpeedLimiter> limiter_lin_ = nullptr;
  std::shared_ptr<SpeedLimiter> limiter_ang_ = nullptr;

  // Publish limited velocity
  bool publish_cmd_;

  // Previous time
  rclcpp::Time previous_time_;
};

} // namespace diff_drive_controller

#endif // DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_