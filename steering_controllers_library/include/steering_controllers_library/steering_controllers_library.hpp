// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "steering_controllers_library/visibility_control.h"

// TODO(anyone): Replace with controller specific messages
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "steering_controllers_library/steering_controllers_library_parameters.hpp"
#include "steering_controllers_library/steering_odometry.hpp"

namespace steering_controllers_library
{
class SteeringControllersLibrary : public controller_interface::ChainableControllerInterface
{
public:
  STEERING_CONTROLLERS__VISIBILITY_PUBLIC SteeringControllersLibrary();

  virtual STEERING_CONTROLLERS__VISIBILITY_PUBLIC void
  initialize_implementation_parameter_listener() = 0;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  virtual STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry() = 0;

  virtual STEERING_CONTROLLERS__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) = 0;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::return_type
  update_reference_from_subscribers() override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::return_type
  update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerAckermannReferenceMsg = ackermann_msgs::msg::AckermannDriveStamped;
  using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
  using AckermannControllerState = control_msgs::msg::SteeringControllerStatus;

protected:
  controller_interface::CallbackReturn set_interface_numbers(
    size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);

  std::shared_ptr<steering_controllers_library::ParamListener> param_listener_;
  steering_controllers_library::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_twist_ = nullptr;
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_ackermann_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_subscriber_unstamped_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerTwistReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;

  std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
  std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  /// Odometry:
  steering_odometry::SteeringOdometry odometry_;

  AckermannControllerState published_state_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<AckermannControllerState>;
  rclcpp::Publisher<AckermannControllerState>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // name constants for state interfaces
  size_t nr_state_itfs_;
  // name constants for command interfaces
  size_t nr_cmd_itfs_;
  // name constants for reference interfaces
  size_t nr_ref_itfs_;

  // last velocity commands for open loop odometry
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;

  std::vector<std::string> rear_wheels_state_names_;
  std::vector<std::string> front_wheels_state_names_;

private:
  // callback for topic interface
  STEERING_CONTROLLERS__VISIBILITY_LOCAL void reference_callback(
    const std::shared_ptr<ControllerTwistReferenceMsg> msg);
  void reference_callback_unstamped(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
};

}  // namespace steering_controllers_library

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_
