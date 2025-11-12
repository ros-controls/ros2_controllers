// Copyright (c) 2025, b-robotized Group
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

#ifndef BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_
#define BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <battery_state_broadcaster/battery_state_broadcaster_parameters.hpp>
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/battery_state_array.hpp"

namespace battery_state_broadcaster
{
/**
 * \brief Battery State Broadcaster for all or some state in a ros2_control system.
 *
 * BatteryStateBroadcaster publishes state interfaces from ros2_control as ROS messages.
 * The following state interfaces can be published:
 *    <state_joint>/battery_voltage (Mandatory)
 *    <state_joint>/battery_temperature
 *    <state_joint>/battery_current
 *    <state_joint>/battery_charge
 *    <state_joint>/battery_percentage
 *    <state_joint>/battery_power_supply_status
 *    <state_joint>/battery_power_supply_health
 *    <state_joint>/battery_present
 *
 * \param state_joints of the batteries to publish.
 * \param capacity of the batteries to publish.
 * \param design_capacity of the batteries to publish.
 * \param power_supply_technology of the batteries to publish.
 * \param location of the batteries to publish.
 * \param serial_number of the batteries to publish.
 *
 * Publishes to:
 *
 * - \b battery_state (sensor_msgs::msg::BatteryState): battery state of the combined battery
 * joints.
 * - \b raw_battery_states (battery_state_broadcaster::msg::BatteryStateArray): battery states of
 * the individual battery joints.
 *
 */
class BatteryStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  BatteryStateBroadcaster();

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
  battery_state_broadcaster::Params params_;

  std::vector<std::string> state_joints_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>
    battery_state_realtime_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryStateArray>>
    raw_battery_states_realtime_publisher_;
  struct BatteryInterfaceSums
  {
    float voltage_sum = 0.0f;
    float temperature_sum = 0.0f;
    float current_sum = 0.0f;
    float charge_sum = 0.0f;
    float percentage_sum = 0.0f;
    float capacity_sum = 0.0f;
    float design_capacity_sum = 0.0f;
  };

  struct BatteryInterfaceCounts
  {
    float temperature_cnt = 0.0f;
    float current_cnt = 0.0f;
    float percentage_cnt = 0.0f;
  };

  BatteryInterfaceSums sums_;
  BatteryInterfaceCounts counts_;

private:
  std::shared_ptr<battery_state_broadcaster::ParamListener> param_listener_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryStateArray>>
    raw_battery_states_publisher_;

  std::vector<bool> battery_presence_;
};

}  // namespace battery_state_broadcaster

#endif  // BATTERY_STATE_BROADCASTER__BATTERY_STATE_BROADCASTER_HPP_
