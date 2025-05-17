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

#include "force_torque_sensor_broadcaster/force_torque_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace force_torque_sensor_broadcaster
{
ForceTorqueSensorBroadcaster::ForceTorqueSensorBroadcaster()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn ForceTorqueSensorBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceTorqueSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  const bool no_interface_names_defined =
    params_.interface_names.force.x.empty() && params_.interface_names.force.y.empty() &&
    params_.interface_names.force.z.empty() && params_.interface_names.torque.x.empty() &&
    params_.interface_names.torque.y.empty() && params_.interface_names.torque.z.empty();

  if (params_.sensor_name.empty() && no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'sensor_name' or at least one "
      "'interface_names.[force|torque].[x|y|z]' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.sensor_name.empty() && !no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "both 'sensor_name' and "
      "'interface_names.[force|torque].[x|y|z]' parameters can not be specified together.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.sensor_name.empty())
  {
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(params_.sensor_name));

    // TODO(juliaj): remove the logging after resolving
    // https://github.com/ros-controls/ros2_controllers/issues/1574
    RCLCPP_INFO(
      get_node()->get_logger(), "Initialized force_torque_sensor with sensor name %s",
      params_.sensor_name.c_str());
  }
  else
  {
    auto const & force_names = params_.interface_names.force;
    auto const & torque_names = params_.interface_names.torque;
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(
        force_names.x, force_names.y, force_names.z, torque_names.x, torque_names.y,
        torque_names.z));

    // TODO(juliaj): remove the logging after resolving
    // https://github.com/ros-controls/ros2_controllers/issues/1574
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Initialized force_torque_sensor with interface names %s, %s, %s, %s, %s, %s",
      force_names.x.c_str(), force_names.y.c_str(), force_names.z.c_str(), torque_names.x.c_str(),
      torque_names.y.c_str(), torque_names.z.c_str());
  }

  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(juliaj): remove the logging after resolving
  // https://github.com/ros-controls/ros2_controllers/issues/1574
  RCLCPP_INFO(get_node()->get_logger(), "Locking realtime publisher");
  realtime_publisher_->lock();
  RCLCPP_INFO(get_node()->get_logger(), "Locked realtime publisher");

  realtime_publisher_->msg_.header.frame_id = params_.frame_id;

  RCLCPP_INFO(get_node()->get_logger(), "Unlocking realtime publisher");
  realtime_publisher_->unlock();
  RCLCPP_INFO(get_node()->get_logger(), "Unlocked realtime publisher");

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ForceTorqueSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn ForceTorqueSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceTorqueSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueSensorBroadcaster::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    force_torque_sensor_->get_values_as_message(realtime_publisher_->msg_.wrench);
    this->apply_sensor_offset(params_, realtime_publisher_->msg_);
    this->apply_sensor_multiplier(params_, realtime_publisher_->msg_);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ForceTorqueSensorBroadcaster::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ForceTorqueSensorBroadcaster::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_state_interfaces;

  std::vector<std::string> force_names(
    {params_.interface_names.force.x, params_.interface_names.force.y,
     params_.interface_names.force.z});
  std::vector<std::string> torque_names(
    {params_.interface_names.torque.x, params_.interface_names.torque.y,
     params_.interface_names.torque.z});
  std::string export_prefix = get_node()->get_name();
  if (!params_.sensor_name.empty())
  {
    const auto semantic_comp_itf_names = force_torque_sensor_->get_state_interface_names();
    std::copy(
      semantic_comp_itf_names.begin(), semantic_comp_itf_names.begin() + 3, force_names.begin());
    std::copy(
      semantic_comp_itf_names.begin() + 3, semantic_comp_itf_names.end(), torque_names.begin());

    // Update the prefix and get the proper force and torque names
    export_prefix = export_prefix + "/" + params_.sensor_name;
    // strip "/" and get the second part of the information
    // e.g. /ft_sensor/force.x -> force.x
    std::for_each(
      force_names.begin(), force_names.end(),
      [](std::string & name) { name = name.substr(name.find_last_of("/") + 1); });
    std::for_each(
      torque_names.begin(), torque_names.end(),
      [](std::string & name) { name = name.substr(name.find_last_of("/") + 1); });
  }
  if (!force_names[0].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, force_names[0], &realtime_publisher_->msg_.wrench.force.x));
  }
  if (!force_names[1].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, force_names[1], &realtime_publisher_->msg_.wrench.force.y));
  }
  if (!force_names[2].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, force_names[2], &realtime_publisher_->msg_.wrench.force.z));
  }
  if (!torque_names[0].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[0], &realtime_publisher_->msg_.wrench.torque.x));
  }
  if (!torque_names[1].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[1], &realtime_publisher_->msg_.wrench.torque.y));
  }
  if (!torque_names[2].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[2], &realtime_publisher_->msg_.wrench.torque.z));
  }
  return exported_state_interfaces;
}

void ForceTorqueSensorBroadcaster::apply_sensor_offset(
  const Params & params, geometry_msgs::msg::WrenchStamped & msg)
{
  msg.wrench.force.x += params.offset.force.x;
  msg.wrench.force.y += params.offset.force.y;
  msg.wrench.force.z += params.offset.force.z;
  msg.wrench.torque.x += params.offset.torque.x;
  msg.wrench.torque.y += params.offset.torque.y;
  msg.wrench.torque.z += params.offset.torque.z;
}

void ForceTorqueSensorBroadcaster::apply_sensor_multiplier(
  const Params & params, geometry_msgs::msg::WrenchStamped & msg)
{
  msg.wrench.force.x *= params.multiplier.force.x;
  msg.wrench.force.y *= params.multiplier.force.y;
  msg.wrench.force.z *= params.multiplier.force.z;
  msg.wrench.torque.x *= params.multiplier.torque.x;
  msg.wrench.torque.y *= params.multiplier.torque.y;
  msg.wrench.torque.z *= params.multiplier.torque.z;
}
}  // namespace force_torque_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_torque_sensor_broadcaster::ForceTorqueSensorBroadcaster,
  controller_interface::ChainableControllerInterface)
