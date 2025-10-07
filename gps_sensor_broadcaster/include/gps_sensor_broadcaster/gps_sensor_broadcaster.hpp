// Copyright 2025 ros2_control development team
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
 * Authors: Wiktor Bajor, Jakub Delicat
 */

#ifndef GPS_SENSOR_BROADCASTER__GPS_SENSOR_BROADCASTER_HPP_
#define GPS_SENSOR_BROADCASTER__GPS_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "gps_sensor_broadcaster/gps_sensor_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/gps_sensor.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace gps_sensor_broadcaster
{
using callback_return_type =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class GPSSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  callback_return_type on_init() override;
  callback_return_type on_configure(const rclcpp_lifecycle::State & previous_state) override;
  callback_return_type on_activate(const rclcpp_lifecycle::State & previous_state) override;
  callback_return_type on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  using GPSSensorOption = semantic_components::GPSSensorOption;
  using GPSSensorVariant = std::variant<
    std::monostate, semantic_components::GPSSensor<GPSSensorOption::WithCovariance>,
    semantic_components::GPSSensor<GPSSensorOption::WithoutCovariance>>;
  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::NavSatFix>;

  void setup_covariance();
  callback_return_type setup_publisher();

  GPSSensorVariant gps_sensor_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  sensor_msgs::msg::NavSatFix state_message_;

  std::shared_ptr<gps_sensor_broadcaster::ParamListener> param_listener_{};
  gps_sensor_broadcaster::Params params_;
  std::vector<std::string> state_names_;
};

}  // namespace gps_sensor_broadcaster

#endif  // GPS_SENSOR_BROADCASTER__GPS_SENSOR_BROADCASTER_HPP_
