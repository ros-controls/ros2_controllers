// Copyright 2026 Christian Rauch
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
 * Authors: Christian Rauch, Subhas Das, Denis Stogl, Victor Lopez
 */

#ifndef MAGNETOMETER_BROADCASTER__MAGNETOMETER_BROADCASTER_HPP_
#define MAGNETOMETER_BROADCASTER__MAGNETOMETER_BROADCASTER_HPP_

#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <magnetometer_broadcaster/magnetometer_broadcaster_parameters.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <semantic_components/magnetic_field_sensor.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace magnetometer_broadcaster
{

class MagnetometerBroadcaster : public controller_interface::ControllerInterface
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

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<semantic_components::MagneticFieldSensor> magnetometer_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  sensor_msgs::msg::MagneticField state_message_;
};

}  // namespace magnetometer_broadcaster

#endif  // MAGNETOMETER_BROADCASTER__MAGNETOMETER_BROADCASTER_HPP_
