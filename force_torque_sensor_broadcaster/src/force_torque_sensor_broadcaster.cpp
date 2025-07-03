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

#include <limits>
#include <memory>
#include <string>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  }
  else
  {
    auto const & force_names = params_.interface_names.force;
    auto const & torque_names = params_.interface_names.torque;
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(
        force_names.x, force_names.y, force_names.z, torque_names.x, torque_names.y,
        torque_names.z));
  }

  try
  {
    filter_chain_ =
      std::make_unique<filters::FilterChain<WrenchMsgType>>("geometry_msgs::msg::WrenchStamped");
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr,
      "Exception thrown during filter chain creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  if (!filter_chain_->configure(
        "sensor_filter_chain", get_node()->get_node_logging_interface(),
        get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not configure sensor filter chain, please check if the "
      "parameters are provided correctly.");
    return CallbackReturn::ERROR;
  }
  try
  {
    // register ft sensor data publisher
    sensor_raw_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/wrench", rclcpp::SystemDefaultsQoS());
    realtime_raw_publisher_ = std::make_unique<StateRTPublisher>(sensor_raw_state_publisher_);

    sensor_filtered_state_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "~/wrench_filtered", rclcpp::SystemDefaultsQoS());
    realtime_filtered_publisher_ =
      std::make_unique<StateRTPublisher>(sensor_filtered_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  wrench_raw_.header.frame_id = params_.frame_id;
  wrench_filtered_.header.frame_id = params_.frame_id;

  realtime_raw_publisher_->lock();
  realtime_raw_publisher_->msg_.header.frame_id = params_.frame_id;
  realtime_raw_publisher_->unlock();

  realtime_filtered_publisher_->lock();
  realtime_filtered_publisher_->msg_.header.frame_id = params_.frame_id;
  realtime_filtered_publisher_->unlock();

  // Add additional frames to publish if any exits
  if (!params_.additional_frames_to_publish.empty())
  {
    auto nr_frames = params_.additional_frames_to_publish.size();
    wrench_additional_frames_pubs_.reserve(nr_frames);
    wrench_additional_frames_publishers_.reserve(nr_frames);
    for (const auto & frame : params_.additional_frames_to_publish)
    {
      StatePublisher pub = get_node()->create_publisher<WrenchMsgType>(
        "~/wrench_filtered_" + frame, rclcpp::SystemDefaultsQoS());
      wrench_additional_frames_pubs_.emplace_back(pub);
      wrench_additional_frames_publishers_.emplace_back(std::make_unique<StateRTPublisher>(pub));

      wrench_additional_frames_publishers_.back()->lock();
      wrench_additional_frames_publishers_.back()->msg_.header.frame_id = frame;
      wrench_additional_frames_publishers_.back()->unlock();
    }

    // initialize buffer transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
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
  param_listener_->try_get_params(params_);

  wrench_raw_.header.stamp = time;
  force_torque_sensor_->get_values_as_message(wrench_raw_.wrench);
  this->apply_sensor_offset(params_, wrench_raw_);
  this->apply_sensor_multiplier(params_, wrench_raw_);

  if (realtime_raw_publisher_ && realtime_raw_publisher_->trylock())
  {
    realtime_raw_publisher_->msg_.header.stamp = time;
    realtime_raw_publisher_->msg_.wrench = wrench_raw_.wrench;
    realtime_raw_publisher_->unlockAndPublish();
  }

  // Filter sensor data
  auto filtered = filter_chain_->update(wrench_raw_, wrench_filtered_);
  if (!filtered)
  {
    wrench_filtered_.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
    wrench_filtered_.wrench.force.y = std::numeric_limits<double>::quiet_NaN();
    wrench_filtered_.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
    wrench_filtered_.wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
    wrench_filtered_.wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
    wrench_filtered_.wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
  }

  if (realtime_filtered_publisher_ && realtime_filtered_publisher_->trylock())
  {
    realtime_filtered_publisher_->msg_.header.stamp = time;
    realtime_filtered_publisher_->msg_.wrench = wrench_filtered_.wrench;
    realtime_filtered_publisher_->unlockAndPublish();
  }

  for (const auto & publisher : wrench_additional_frames_publishers_)
  {
    try
    {
      if (filtered)
      {
        auto transform = tf_buffer_->lookupTransform(
          publisher->msg_.header.frame_id, params_.frame_id, tf2::TimePointZero);
        tf2::doTransform(wrench_filtered_, publisher->msg_, transform);
      }
      else
      {
        throw tf2::TransformException("cannot transform, filtered message is invalid");
      }
    }
    catch (const tf2::TransformException & e)
    {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 5000,
        "LookupTransform failed from '%s' to '%s'. %s", params_.frame_id.c_str(),
        publisher->msg_.header.frame_id.c_str(), e.what());
      publisher->msg_.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.force.y = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
    }
    if (publisher && publisher->trylock())
    {
      publisher->msg_.header.stamp = time;
      publisher->unlockAndPublish();
    }
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
        export_prefix, force_names[0], &realtime_raw_publisher_->msg_.wrench.force.x));
  }
  if (!force_names[1].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, force_names[1], &realtime_raw_publisher_->msg_.wrench.force.y));
  }
  if (!force_names[2].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, force_names[2], &realtime_raw_publisher_->msg_.wrench.force.z));
  }
  if (!torque_names[0].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[0], &realtime_raw_publisher_->msg_.wrench.torque.x));
  }
  if (!torque_names[1].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[1], &realtime_raw_publisher_->msg_.wrench.torque.y));
  }
  if (!torque_names[2].empty())
  {
    exported_state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        export_prefix, torque_names[2], &realtime_raw_publisher_->msg_.wrench.torque.z));
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
