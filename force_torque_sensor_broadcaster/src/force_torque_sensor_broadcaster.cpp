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
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace force_torque_sensor_broadcaster
{
ForceTorqueSensorBroadcaster::ForceTorqueSensorBroadcaster()
: controller_interface::ControllerInterface()
{
}

CallbackReturn ForceTorqueSensorBroadcaster::on_init()
{
  try
  {
    get_node()->declare_parameter<std::string>("sensor_name", "");
    get_node()->declare_parameter<std::string>("interface_names.force.x", "");
    get_node()->declare_parameter<std::string>("interface_names.force.y", "");
    get_node()->declare_parameter<std::string>("interface_names.force.z", "");
    get_node()->declare_parameter<std::string>("interface_names.torque.x", "");
    get_node()->declare_parameter<std::string>("interface_names.torque.y", "");
    get_node()->declare_parameter<std::string>("interface_names.torque.z", "");
    get_node()->declare_parameter<std::string>("frame_id", "");
    get_node()->declare_parameter<std::vector<std::string>>(
      "additional_frames_to_publish", std::vector<std::string>({}));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  interface_names_[0] = get_node()->get_parameter("interface_names.force.x").as_string();
  interface_names_[1] = get_node()->get_parameter("interface_names.force.y").as_string();
  interface_names_[2] = get_node()->get_parameter("interface_names.force.z").as_string();
  interface_names_[3] = get_node()->get_parameter("interface_names.torque.x").as_string();
  interface_names_[4] = get_node()->get_parameter("interface_names.torque.y").as_string();
  interface_names_[5] = get_node()->get_parameter("interface_names.torque.z").as_string();

  const bool no_interface_names_defined =
    std::count(interface_names_.begin(), interface_names_.end(), "") == 6;

  if (sensor_name_.empty() && no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'sensor_name' or at least one "
      "'interface_names.[force|torque].[x|y|z]' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty() && !no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "both 'sensor_name' and "
      "'interface_names.[force|torque].[x|y|z]' parameters can not be specified together.");
    return CallbackReturn::ERROR;
  }

  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  if (frame_id_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be provided.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty())
  {
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(sensor_name_));
  }
  else
  {
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      semantic_components::ForceTorqueSensor(
        interface_names_[0], interface_names_[1], interface_names_[2], interface_names_[3],
        interface_names_[4], interface_names_[5]));
  }

  try
  {
    // register ft sensor data publishers
    wrench_raw_pub_ =
      get_node()->create_publisher<WrenchMsgType>("~/wrench", rclcpp::SystemDefaultsQoS());
    wrench_raw_publisher_ = std::make_unique<WrenchRTPublisher>(wrench_raw_pub_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  wrench_raw_.header.frame_id = frame_id_;

  wrench_raw_publisher_->lock();
  wrench_raw_publisher_->msg_.header.frame_id = frame_id_;
  wrench_raw_publisher_->unlock();

  // Add additional frames to publish if any exits
  additional_frames_to_publish_ =
    get_node()->get_parameter("additional_frames_to_publish").as_string_array();

  if (!additional_frames_to_publish_.empty())
  {
    auto nr_frames = additional_frames_to_publish_.size();
    wrench_aditional_frames_pubs_.reserve(nr_frames);
    wrench_aditional_frames_publishers_.reserve(nr_frames);
    for (const auto & frame : additional_frames_to_publish_)
    {
      WrenchPublisher pub = get_node()->create_publisher<WrenchMsgType>(
        "~/wrench_" + frame, rclcpp::SystemDefaultsQoS());
      wrench_aditional_frames_pubs_.emplace_back(pub);
      wrench_aditional_frames_publishers_.emplace_back(std::make_unique<WrenchRTPublisher>(pub));

      wrench_aditional_frames_publishers_.back()->lock();
      wrench_aditional_frames_publishers_.back()->msg_.header.frame_id = frame;
      wrench_aditional_frames_publishers_.back()->unlock();
    }

    // initialize buffer transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
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

CallbackReturn ForceTorqueSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  force_torque_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  wrench_raw_.header.stamp = time;
  force_torque_sensor_->get_values_as_message(wrench_raw_.wrench);

  // Publish sensor data
  if (wrench_raw_publisher_ && wrench_raw_publisher_->trylock())
  {
    wrench_raw_publisher_->msg_.header.stamp = time;
    wrench_raw_publisher_->msg_.wrench = wrench_raw_.wrench;
    wrench_raw_publisher_->unlockAndPublish();
  }

  for (const auto & publisher : wrench_aditional_frames_publishers_)
  {
    try
    {
      auto transform =
        tf_buffer_->lookupTransform(publisher->msg_.header.frame_id, frame_id_, tf2::TimePointZero);
      tf2::doTransform(wrench_raw_, publisher->msg_, transform);

      if (publisher && publisher->trylock())
      {
        publisher->msg_.header.stamp = time;
        publisher->unlockAndPublish();
      }
    }
    catch (const tf2::TransformException & e)
    {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 5000,
        "LookupTransform failed from '%s' to '%s'.", frame_id_.c_str(),
        publisher->msg_.header.frame_id.c_str());
      publisher->msg_.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.force.y = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
      publisher->msg_.wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace force_torque_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  force_torque_sensor_broadcaster::ForceTorqueSensorBroadcaster,
  controller_interface::ControllerInterface)
