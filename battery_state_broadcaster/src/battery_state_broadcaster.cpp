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

#include "battery_state_broadcaster/battery_state_broadcaster.hpp"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace battery_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
const size_t MAX_LENGTH = 64;  // maximum length of strings to reserve

BatteryStateBroadcaster::BatteryStateBroadcaster() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<battery_state_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (!params_.sensor_name.empty())
  {
    if (params_.state_joints.size() > 0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "You cannot use both 'sensor_name' and 'state_joints' parameters. Please use only "
        "'state_joints' going forward.");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_WARN(
      get_node()->get_logger(),
      "The 'sensor_name' parameter is deprecated and will be removed in future releases. Please "
      "use 'state_joints' parameter instead.");
    state_joints_ = {params_.sensor_name};
  }
  else
  {
    state_joints_ = params_.state_joints;
  }
  battery_presence_.resize(params_.state_joints.size(), false);

  try
  {
    battery_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
      "~/battery_state", rclcpp::SystemDefaultsQoS());

    battery_state_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>(
        battery_state_publisher_);

    raw_battery_states_publisher_ =
      get_node()->create_publisher<control_msgs::msg::BatteryStateArray>(
        "~/raw_battery_states", rclcpp::SystemDefaultsQoS());

    raw_battery_states_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::BatteryStateArray>>(
        raw_battery_states_publisher_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher depending on the message type
  battery_state_msg.location.reserve(MAX_LENGTH);
  battery_state_msg.serial_number.reserve(MAX_LENGTH);

  raw_battery_states_msg.battery_states.reserve(params_.state_joints.size());
  for (size_t i = 0; i < params_.state_joints.size(); ++i)
  {
    sensor_msgs::msg::BatteryState battery;
    battery.location.reserve(MAX_LENGTH);
    battery.serial_number.reserve(MAX_LENGTH);
    raw_battery_states_msg.battery_states.emplace_back(std::move(battery));
  }

  // Get count of enabled joints for each interface
  for (size_t i = 0; i < params_.state_joints.size(); ++i)
  {
    const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));
    const auto & battery_properties = params_.state_joints_map.at(params_.state_joints.at(i));

    if (interfaces.battery_temperature)
    {
      counts_.temperature_cnt++;
    }
    if (interfaces.battery_current)
    {
      counts_.current_cnt++;
    }
    if (interfaces.battery_percentage)
    {
      counts_.percentage_cnt++;
    }
    else
    {
      auto min_volt = battery_properties.minimum_voltage;
      auto max_volt = battery_properties.maximum_voltage;
      if ((!std::isnan(min_volt)) && (!std::isnan(max_volt)))
      {
        if (min_volt >= max_volt)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Maximum battery voltage level must be greater than minimum voltage level.");
          return controller_interface::CallbackReturn::ERROR;
        }
        counts_.percentage_cnt++;
      }
    }
    sums_.capacity_sum += static_cast<float>(battery_properties.capacity);
    sums_.design_capacity_sum += static_cast<float>(battery_properties.design_capacity);
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (!params_.sensor_name.empty())
  {
    state_interfaces_config.names.reserve(1);
    state_interfaces_config.names.push_back(params_.sensor_name + "/voltage");
    return state_interfaces_config;
  }

  state_interfaces_config.names.reserve(params_.state_joints.size() * 8);
  for (const auto & joint : params_.state_joints)
  {
    const auto & interfaces = params_.interfaces.state_joints_map.at(joint);
    state_interfaces_config.names.push_back(joint + "/battery_voltage");
    if (interfaces.battery_temperature)
    {
      state_interfaces_config.names.push_back(joint + "/battery_temperature");
    }
    if (interfaces.battery_current)
    {
      state_interfaces_config.names.push_back(joint + "/battery_current");
    }
    if (interfaces.battery_charge)
    {
      state_interfaces_config.names.push_back(joint + "/battery_charge");
    }
    if (interfaces.battery_percentage)
    {
      state_interfaces_config.names.push_back(joint + "/battery_percentage");
    }
    if (interfaces.battery_power_supply_status)
    {
      state_interfaces_config.names.push_back(joint + "/battery_power_supply_status");
    }
    if (interfaces.battery_power_supply_health)
    {
      state_interfaces_config.names.push_back(joint + "/battery_power_supply_health");
    }
    if (interfaces.battery_present)
    {
      state_interfaces_config.names.push_back(joint + "/battery_present");
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces found to publish.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // get parameters from the listener in case they were updated
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  uint8_t combined_power_supply_technology;
  if (!params_.sensor_name.empty())
  {
    sums_.design_capacity_sum = static_cast<float>(params_.design_capacity);
    combined_power_supply_technology = static_cast<uint8_t>(params_.power_supply_technology);
  }
  else
  {
    combined_power_supply_technology = static_cast<uint8_t>(
      params_.state_joints_map.at(params_.state_joints.at(0)).power_supply_technology);
  }
  std::string combined_location = "";
  std::string combined_serial_number = "";

  // handle individual battery states initializations
  for (size_t i = 0; i < params_.state_joints.size(); ++i)
  {
    auto & battery_state = raw_battery_states_msg.battery_states[i];
    const auto & battery_properties = params_.state_joints_map.at(params_.state_joints.at(i));

    battery_state.header.frame_id = params_.state_joints[i];
    battery_state.voltage = kUninitializedValue;
    battery_state.temperature = kUninitializedValue;
    battery_state.current = kUninitializedValue;
    battery_state.charge = kUninitializedValue;
    battery_state.capacity = static_cast<float>(battery_properties.capacity);
    battery_state.design_capacity = static_cast<float>(battery_properties.design_capacity);
    battery_state.percentage = kUninitializedValue;
    battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology =
      static_cast<uint8_t>(battery_properties.power_supply_technology);
    battery_state.present = true;
    battery_state.cell_voltage = {};
    battery_state.cell_temperature = {};
    battery_state.location = battery_properties.location;
    battery_state.serial_number = battery_properties.serial_number;

    if (combined_power_supply_technology != battery_state.power_supply_technology)
    {
      combined_power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    }
    combined_location += battery_state.location + ", ";
    combined_serial_number += battery_state.serial_number + ", ";
  }
  raw_battery_states_realtime_publisher_->try_publish(raw_battery_states_msg);

  // handle aggregate battery state initialization
  battery_state_msg.voltage = kUninitializedValue;
  battery_state_msg.temperature = kUninitializedValue;
  battery_state_msg.current = kUninitializedValue;
  battery_state_msg.charge = kUninitializedValue;
  battery_state_msg.capacity = sums_.capacity_sum;
  battery_state_msg.design_capacity = sums_.design_capacity_sum;
  battery_state_msg.percentage = kUninitializedValue;
  battery_state_msg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_msg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_msg.power_supply_technology = combined_power_supply_technology;
  battery_state_msg.present = true;
  battery_state_msg.cell_voltage = {};
  battery_state_msg.cell_temperature = {};
  battery_state_msg.location = combined_location;
  battery_state_msg.serial_number = combined_serial_number;

  battery_state_realtime_publisher_->try_publish(battery_state_msg);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sums_ = {};
  int interface_cnt = 0;
  uint8_t combined_power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  uint8_t combined_power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

  if (raw_battery_states_realtime_publisher_)
  {
    for (size_t i = 0; i < params_.state_joints.size(); ++i)
    {
      const auto & interfaces = params_.interfaces.state_joints_map.at(params_.state_joints.at(i));

      raw_battery_states_msg.battery_states[i].header.stamp = time;

      raw_battery_states_msg.battery_states[i].voltage = static_cast<float>(
        state_interfaces_[interface_cnt].get_optional<double>().value_or(kUninitializedValue));
      sums_.voltage_sum += raw_battery_states_msg.battery_states[i].voltage;
      interface_cnt++;

      if (interfaces.battery_temperature)
      {
        raw_battery_states_msg.battery_states[i].temperature = static_cast<float>(
          state_interfaces_[interface_cnt].get_optional<double>().value_or(kUninitializedValue));
        sums_.temperature_sum += raw_battery_states_msg.battery_states[i].temperature;
        interface_cnt++;
      }
      if (interfaces.battery_current)
      {
        raw_battery_states_msg.battery_states[i].current = static_cast<float>(
          state_interfaces_[interface_cnt].get_optional<double>().value_or(kUninitializedValue));
        sums_.current_sum += raw_battery_states_msg.battery_states[i].current;
        interface_cnt++;
      }
      if (interfaces.battery_charge)
      {
        raw_battery_states_msg.battery_states[i].charge = static_cast<float>(
          state_interfaces_[interface_cnt].get_optional<double>().value_or(kUninitializedValue));
        sums_.charge_sum += raw_battery_states_msg.battery_states[i].charge;
        interface_cnt++;
      }
      if (interfaces.battery_percentage)
      {
        raw_battery_states_msg.battery_states[i].percentage = static_cast<float>(
          state_interfaces_[interface_cnt].get_optional<double>().value_or(kUninitializedValue));
        sums_.percentage_sum += raw_battery_states_msg.battery_states[i].percentage;
        interface_cnt++;
      }
      else
      {
        auto min_volt = params_.state_joints_map.at(params_.state_joints.at(i)).minimum_voltage;
        auto max_volt = params_.state_joints_map.at(params_.state_joints.at(i)).maximum_voltage;
        float voltage = raw_battery_states_msg.battery_states[i].voltage;

        raw_battery_states_msg.battery_states[i].percentage =
          static_cast<float>((voltage - min_volt) * 100.0 / (max_volt - min_volt));
        sums_.percentage_sum += raw_battery_states_msg.battery_states[i].percentage;
      }
      if (interfaces.battery_power_supply_status)
      {
        raw_battery_states_msg.battery_states[i].power_supply_status =
          static_cast<uint8_t>(state_interfaces_[interface_cnt].get_optional<double>().value_or(
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN));
        if (
          raw_battery_states_msg.battery_states[i].power_supply_status >
          combined_power_supply_status)
        {
          combined_power_supply_status =
            raw_battery_states_msg.battery_states[i].power_supply_status;
        }
        interface_cnt++;
      }
      if (interfaces.battery_power_supply_health)
      {
        raw_battery_states_msg.battery_states[i].power_supply_health =
          static_cast<uint8_t>(state_interfaces_[interface_cnt].get_optional<double>().value_or(
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN));
        if (
          raw_battery_states_msg.battery_states[i].power_supply_health >
          combined_power_supply_health)
        {
          combined_power_supply_health =
            raw_battery_states_msg.battery_states[i].power_supply_health;
        }
        interface_cnt++;
      }
      if (interfaces.battery_present)
      {
        raw_battery_states_msg.battery_states[i].present =
          state_interfaces_[interface_cnt].get_optional().value_or(false);
        interface_cnt++;
      }
      else
      {
        if (
          (!std::isnan(raw_battery_states_msg.battery_states[i].voltage)) &&
          (raw_battery_states_msg.battery_states[i].voltage))
        {
          raw_battery_states_msg.battery_states[i].present = true;
        }
        else
        {
          raw_battery_states_msg.battery_states[i].present = false;
        }
      }
    }
    raw_battery_states_realtime_publisher_->try_publish(raw_battery_states_msg);
  }

  if (!params_.sensor_name.empty())
  {
    sums_.voltage_sum =
      static_cast<float>(state_interfaces_[0].get_optional<double>().value_or(kUninitializedValue));
  }

  if (battery_state_realtime_publisher_)
  {
    battery_state_msg.header.stamp = time;
    battery_state_msg.voltage = sums_.voltage_sum / static_cast<float>(state_joints_.size());

    if (counts_.temperature_cnt)
    {
      battery_state_msg.temperature = sums_.temperature_sum / counts_.temperature_cnt;
    }
    if (counts_.current_cnt)
    {
      battery_state_msg.current = sums_.current_sum / counts_.current_cnt;
    }
    battery_state_msg.charge = sums_.charge_sum;
    if (counts_.percentage_cnt)
    {
      battery_state_msg.percentage = sums_.percentage_sum / counts_.percentage_cnt;
    }
    battery_state_msg.power_supply_status = combined_power_supply_status;
    battery_state_msg.power_supply_health = combined_power_supply_health;

    battery_state_realtime_publisher_->try_publish(battery_state_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace battery_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  battery_state_broadcaster::BatteryStateBroadcaster, controller_interface::ControllerInterface)
