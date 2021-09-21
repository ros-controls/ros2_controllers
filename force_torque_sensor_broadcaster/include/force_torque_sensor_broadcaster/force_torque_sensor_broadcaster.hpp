// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
 * Authors: Subhas Das, Denis Stogl
 */

#ifndef FORCE_TORQUE_SENSOR_BROADCASTER__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
#define FORCE_TORQUE_SENSOR_BROADCASTER__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "force_torque_sensor_broadcaster/visibility_control.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"

namespace force_torque_sensor_broadcaster
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ForceTorqueSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  ForceTorqueSensorBroadcaster();

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC CallbackReturn on_init() override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  FORCE_TORQUE_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string sensor_name_;
  std::array<std::string, 6> interface_names_;
  std::string frame_id_;

  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace force_torque_sensor_broadcaster

#endif  // FORCE_TORQUE_SENSOR_BROADCASTER__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
