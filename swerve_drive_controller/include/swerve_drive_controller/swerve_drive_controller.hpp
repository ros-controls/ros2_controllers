// Copyright 2025 ros2_control development team
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

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace swerve_drive_controller
{

enum class WheelAxleIndex : std::size_t
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

using CallbackReturn = controller_interface::CallbackReturn;

class Wheel
{
public:
  Wheel(
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback,
    std::string name);

  void set_velocity(double velocity);
  double get_feedback();

private:
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_;
  std::string name_;
};

class Axle
{
public:
  Axle(
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback,
    std::string name);

  void set_position(double position);
  double get_feedback();

private:
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_;
  std::string name_;
};

class SwerveController : public controller_interface::ControllerInterface
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Twist = geometry_msgs::msg::Twist;

public:
  SwerveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::unique_ptr<Wheel> get_wheel(const std::string & wheel_name);
  std::unique_ptr<Axle> get_axle(const std::string & axle_name);

  // Handles for four wheels and their axles
  std::vector<std::unique_ptr<Wheel>> wheel_handles_;
  std::vector<std::unique_ptr<Axle>> axle_handles_;

  // Joint names for wheels and axles
  std::string front_left_wheel_joint_name_;
  std::string front_right_wheel_joint_name_;
  std::string rear_left_wheel_joint_name_;
  std::string rear_right_wheel_joint_name_;

  std::string front_left_axle_joint_name_;
  std::string front_right_axle_joint_name_;
  std::string rear_left_axle_joint_name_;
  std::string rear_right_axle_joint_name_;

  std::array<std::string, 4> wheel_joint_names{};
  std::array<std::string, 4> axle_joint_names{};

  std::string cmd_vel_topic_;
  std::string odometry_topic_;
  std::string base_footprint_;

  bool enable_odom_tf_ = true;
  bool open_loop_ = false;
  bool use_stamped_vel_ = false;

  double front_left_velocity_threshold_;
  double front_right_velocity_threshold_;
  double rear_left_velocity_threshold_;
  double rear_right_velocity_threshold_;

  SwerveDriveKinematics swerveDriveKinematics_;

  std::queue<TwistStamped> previous_commands_;  // last two commands

  double pose_covariance_diagonal_array_[6];
  double twist_covariance_diagonal_array_[6];

  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  struct WheelParams
  {
    double x_offset = 0.0;  // Chassis Center to Axle Center
    double y_offset = 0.0;  // Axle Center to Wheel Center
    double radius = 0.0;    // Assumed to be the same for all wheels
    double center_of_rotation = 0.0;
  } wheel_params_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};
  rclcpp::Time previous_update_timestamp_{0};

  // Topic Subscription
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool is_halted_ = false;

  bool reset();
  void halt();
};

}  // namespace swerve_drive_controller
#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
