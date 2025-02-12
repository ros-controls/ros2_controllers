// Copyright 2025 Aarav Gupta
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

#ifndef MULTI_OMNI_WHEEL_DRIVE_CONTROLLER__MULTI_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_
#define MULTI_OMNI_WHEEL_DRIVE_CONTROLLER__MULTI_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "multi_omni_wheel_drive_controller/odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// auto-generated by generate_parameter_library
#include "multi_omni_wheel_drive_controller/multi_omni_wheel_drive_controller_parameters.hpp"

namespace multi_omni_wheel_drive_controller
{
class MultiOmniWheelDriveController : public controller_interface::ChainableControllerInterface
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;

public:
  MultiOmniWheelDriveController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  bool on_set_chained_mode(bool chained_mode) override;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // Parameters from ROS for multi_omni_wheel_drive_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  const char * feedback_type() const;

  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };
  std::vector<WheelHandle> registered_wheel_handles_;

  controller_interface::CallbackReturn configure_wheel_handles(
    const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles);

  // Timeout to consider cmd_vel commands old
  rclcpp::Duration cmd_vel_timeout_ = rclcpp::Duration::from_seconds(0.5);

  bool is_halted_ = false;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};

  void compute_and_set_wheel_velocities();

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool reset();
  void halt();

private:
  void reset_buffers();
};
}  // namespace multi_omni_wheel_drive_controller

#endif  // MULTI_OMNI_WHEEL_DRIVE_CONTROLLER__MULTI_OMNI_WHEEL_DRIVE_CONTROLLER_HPP_
