// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#ifndef IMU_SENSOR_BROADCASTER__IMU_SENSOR_BROADCASTER_HPP_
#define IMU_SENSOR_BROADCASTER__IMU_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "imu_sensor_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu_sensor_broadcaster
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class IMUSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  IMU_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  IMU_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  IMU_SENSOR_BROADCASTER_PUBLIC CallbackReturn on_init() override;

  IMU_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  IMU_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  IMU_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  IMU_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string sensor_name_;
  std::string frame_id_;

  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace imu_sensor_broadcaster

#endif  // IMU_SENSOR_BROADCASTER__IMU_SENSOR_BROADCASTER_HPP_
