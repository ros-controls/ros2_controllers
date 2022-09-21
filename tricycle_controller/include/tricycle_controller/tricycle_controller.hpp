// Copyright 2022 Pixel Robotics.
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
 * Author: Tony Najjar
 */

#ifndef TRICYCLE_CONTROLLER__TRICYCLE_CONTROLLER_HPP_
#define TRICYCLE_CONTROLLER__TRICYCLE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tricycle_controller/odometry.hpp"
#include "tricycle_controller/steering_limiter.hpp"
#include "tricycle_controller/traction_limiter.hpp"
#include "tricycle_controller/visibility_control.h"

namespace tricycle_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TricycleController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using AckermannDrive = ackermann_msgs::msg::AckermannDrive;

public:
  TRICYCLE_CONTROLLER_PUBLIC
  TricycleController();

  TRICYCLE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TRICYCLE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TRICYCLE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  TRICYCLE_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct TractionHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
  };
  struct SteeringHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_command;
  };

  CallbackReturn get_traction(
    const std::string & traction_joint_name, std::vector<TractionHandle> & joint);
  CallbackReturn get_steering(
    const std::string & steering_joint_name, std::vector<SteeringHandle> & joint);
  double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase);
  std::tuple<double, double> twist_to_ackermann(double linear_command, double angular_command);

  std::string traction_joint_name_;
  std::string steering_joint_name_;

  // HACK: put into vector to avoid initializing structs because they have no default constructors
  std::vector<TractionHandle> traction_joint_;
  std::vector<SteeringHandle> steering_joint_;

  struct WheelParams
  {
    double wheelbase = 0.0;
    double radius = 0.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = false;
    bool odom_only_twist = false;  // for doing the pose integration in separate node
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  bool publish_ackermann_command_ = false;
  std::shared_ptr<rclcpp::Publisher<AckermannDrive>> ackermann_command_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<AckermannDrive>>
    realtime_ackermann_command_publisher_ = nullptr;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr_{nullptr};

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_service_;

  std::queue<AckermannDrive> previous_commands_;  // last two commands

  // speed limiters
  TractionLimiter limiter_traction_;
  SteeringLimiter limiter_steering_;

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  void reset_odometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool reset();
  void halt();
};
}  // namespace tricycle_controller
#endif  // TRICYCLE_CONTROLLER__TRICYCLE_CONTROLLER_HPP_
