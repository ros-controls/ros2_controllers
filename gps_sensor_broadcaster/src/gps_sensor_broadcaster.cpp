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

#include "gps_sensor_broadcaster/gps_sensor_broadcaster.hpp"
#include <map>

namespace
{
template <class... Ts>
struct Visitor : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
Visitor(Ts...) -> Visitor<Ts...>;
constexpr std::size_t COVARIANCE_3x3_SIZE = 9;
constexpr uint16_t COVARIANCE_TYPE_KNOWN = 3;
constexpr uint16_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
}  // namespace

namespace gps_sensor_broadcaster
{

callback_return_type GPSSensorBroadcaster::on_init()
try
{
  param_listener_ = std::make_shared<gps_sensor_broadcaster::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}

callback_return_type GPSSensorBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  gps_sensor_ =
    params_.read_covariance_from_interface
      ? GPSSensorVariant{semantic_components::GPSSensor<GPSSensorOption::WithCovariance>(
          params_.sensor_name)}
      : GPSSensorVariant{
          semantic_components::GPSSensor<GPSSensorOption::WithoutCovariance>(params_.sensor_name)};
  std::visit(
    Visitor{
      [this](auto & sensor) { state_names = sensor.get_state_interface_names(); },
      [](std::monostate &) {}},
    gps_sensor_);

  return setup_publisher();
}

callback_return_type GPSSensorBroadcaster::setup_publisher()
try
{
  sensor_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::NavSatFix>(
    "~/gps/fix", rclcpp::SystemDefaultsQoS());
  realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
  setup_covariance();
  realtime_publisher_->unlock();

  return callback_return_type::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(
    stderr, "Exception thrown during publisher creation at configure stage with message: %s \n",
    e.what());
  return callback_return_type::ERROR;
}

void GPSSensorBroadcaster::setup_covariance()
{
  if (params_.read_covariance_from_interface)
  {
    realtime_publisher_->msg_.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
    return;
  }

  for (size_t i = 0; i < COVARIANCE_3x3_SIZE; ++i)
  {
    realtime_publisher_->msg_.position_covariance[i] = params_.static_position_covariance[i];
    realtime_publisher_->msg_.position_covariance_type = COVARIANCE_TYPE_KNOWN;
  }
}

controller_interface::InterfaceConfiguration GPSSensorBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration GPSSensorBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_names;

  return state_interfaces_config;
}

callback_return_type GPSSensorBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  std::visit(
    Visitor{
      [this](auto & sensor) { sensor.assign_loaned_state_interfaces(state_interfaces_); },
      [](std::monostate &) {}},
    gps_sensor_);
  return callback_return_type::SUCCESS;
}

callback_return_type GPSSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::visit(
    Visitor{[](auto & sensor) { sensor.release_interfaces(); }, [](std::monostate &) {}},
    gps_sensor_);
  return callback_return_type::SUCCESS;
}

controller_interface::return_type GPSSensorBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = get_node()->now();
    std::visit(
      Visitor{
        [this](auto & sensor)
        {
          sensor_msgs::msg::NavSatFix message;
          sensor.get_values_as_message(realtime_publisher_->msg_);
        },
        [](std::monostate &) {}},
      gps_sensor_);
    realtime_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
}  // namespace gps_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gps_sensor_broadcaster::GPSSensorBroadcaster, controller_interface::ControllerInterface)
