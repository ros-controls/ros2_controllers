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
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// auto-generated by generate_parameter_library
#include "diff_drive_controller_parameters.hpp"

namespace diff_drive_controller
{
class DiffDriveController : public controller_interface::ControllerInterface
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;

public:
  DiffDriveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char * feedback_type() const;
  controller_interface::CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  // Parameters from ROS for diff_drive_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  /* Number of wheels on each side of the robot. This is important to take the wheels slip into
   * account when multiple wheels on each side are present. If there are more wheels then control
   * signals for each side, you should enter number for control signals. For example, Husky has two
   * wheels on each side, but they use one control signal, in this case '1' is the correct value of
   * the parameter. */
  int wheels_per_side_;

  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};
  std::shared_ptr<TwistStamped> last_command_msg_;

  std::queue<TwistStamped> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistStamped>>
    realtime_limited_velocity_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool is_halted = false;

  bool reset();
  void halt();
};
}  // namespace diff_drive_controller
#endif  // DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
