// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "mecanum_drive_controller/odometry.hpp"
#include "mecanum_drive_controller/visibility_control.h"
#include "mecanum_drive_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "custom_messages/msg/mecanum_drive_controller_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
namespace mecanum_drive_controller
{
// name constants for state interfaces
static constexpr size_t NR_STATE_ITFS = 4;

// name constants for command interfaces
static constexpr size_t NR_CMD_ITFS = 4;

// name constants for reference interfaces
static constexpr size_t NR_REF_ITFS = 3;

class MecanumDriveController : public controller_interface::ChainableControllerInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  MecanumDriveController();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using OdomStateMsg = nav_msgs::msg::Odometry;
  using TfStateMsg = tf2_msgs::msg::TFMessage;
  using ControllerStateMsg = custom_messages::msg::MecanumDriveControllerState;

protected:
  std::shared_ptr<mecanum_drive_controller::ParamListener> param_listener_;
  mecanum_drive_controller::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

  using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_;
  std::unique_ptr<OdomStatePublisher> rt_odom_state_publisher_;

  using TfStatePublisher = realtime_tools::RealtimePublisher<TfStateMsg>;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
  std::unique_ptr<TfStatePublisher> rt_tf_odom_state_publisher_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  // Odometry related:
  Odometry odometry_;

private:
  // callback for topic interface
  TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace mecanum_drive_controller

#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
