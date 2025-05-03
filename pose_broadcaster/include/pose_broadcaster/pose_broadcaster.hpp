// Copyright 2024 FZI Forschungszentrum Informatik
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
#ifndef POSE_BROADCASTER__POSE_BROADCASTER_HPP_
#define POSE_BROADCASTER__POSE_BROADCASTER_HPP_

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/pose_sensor.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "pose_broadcaster/pose_broadcaster_parameters.hpp"

namespace pose_broadcaster
{

class PoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<semantic_components::PoseSensor> pose_sensor_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    realtime_publisher_;

  // TODO(amronos): Remove these two member variables
  std::optional<rclcpp::Duration> tf_publish_period_;
  rclcpp::Time tf_last_publish_time_{0, 0, RCL_CLOCK_UNINITIALIZED};

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_tf_publisher_;
};

}  // namespace pose_broadcaster

#endif  // POSE_BROADCASTER__POSE_BROADCASTER_HPP_
