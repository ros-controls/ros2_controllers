#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include "BatterySensor.hpp"

namespace battery_state_broadcaster
{
class BatteryStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  std::unique_ptr<BatterySensor> battery_sensor_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>> realtime_publisher_;
};
}  // namespace battery_state_broadcaster
