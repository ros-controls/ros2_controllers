// Copyright 2021 ros2_control development team
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

#include "joint_state_broadcaster/joint_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

JointStateBroadcaster::JointStateBroadcaster() {}

CallbackReturn JointStateBroadcaster::on_init()
{
  try
  {
    auto_declare<bool>("use_local_topics", false);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointStateBroadcaster::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

CallbackReturn JointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  use_local_topics_ = get_node()->get_parameter("use_local_topics").as_bool();

  try
  {
    const std::string topic_name_prefix = use_local_topics_ ? "~/" : "";

    joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      topic_name_prefix + "joint_states", rclcpp::SystemDefaultsQoS());

    dynamic_joint_state_publisher_ =
      get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
        topic_name_prefix + "dynamic_joint_states", rclcpp::SystemDefaultsQoS());
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    return CallbackReturn::ERROR;
  }

  init_joint_state_msg();
  init_dynamic_joint_state_msg();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool JointStateBroadcaster::init_joint_data()
{
  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_name()) == 0)
    {
      name_if_value_mapping_[si->get_name()] = {};
    }
    // add interface name
    name_if_value_mapping_[si->get_name()][si->get_interface_name()] = kUninitializedValue;
  }

  // filter state interfaces that have at least one of the joint_states fields,
  // the rest will be ignored for this message
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}))
    {
      joint_names_.push_back(name_ifv.first);
    }
  }

  // Add extra joints from parameters, each joint will be added to joint_names_ and
  // name_if_value_mapping_ if it is not already there
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("extra_joints", extra_joints))
  {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names)
    {
      if (name_if_value_mapping_.count(extra_joint_name) == 0)
      {
        name_if_value_mapping_[extra_joint_name] = {
          {HW_IF_POSITION, 0.0}, {HW_IF_VELOCITY, 0.0}, {HW_IF_EFFORT, 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }

  return true;
}

void JointStateBroadcaster::init_joint_state_msg()
{
  const size_t num_joints = joint_names_.size();

  /// @note joint_state_msg publishes position, velocity and effort for all joints,
  /// with at least one of these interfaces, the rest are omitted from this message

  // default initialization for joint state message
  joint_state_msg_.name = joint_names_;
  joint_state_msg_.position.resize(num_joints, kUninitializedValue);
  joint_state_msg_.velocity.resize(num_joints, kUninitializedValue);
  joint_state_msg_.effort.resize(num_joints, kUninitializedValue);
}

void JointStateBroadcaster::init_dynamic_joint_state_msg()
{
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & name = name_ifv.first;
    const auto & interfaces_and_values = name_ifv.second;
    dynamic_joint_state_msg_.joint_names.push_back(name);
    control_msgs::msg::InterfaceValue if_value;
    for (const auto & interface_and_value : interfaces_and_values)
    {
      if_value.interface_names.emplace_back(interface_and_value.first);
      if_value.values.emplace_back(kUninitializedValue);
    }
    dynamic_joint_state_msg_.interface_values.emplace_back(if_value);
  }
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return kUninitializedValue;
  }
}

controller_interface::return_type JointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    name_if_value_mapping_[state_interface.get_name()][state_interface.get_interface_name()] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_interface_name().c_str(), state_interface.get_value());
  }

  joint_state_msg_.header.stamp = time;
  dynamic_joint_state_msg_.header.stamp = time;

  // update joint state message and dynamic joint state message
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_state_msg_.position[i] =
      get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);
    joint_state_msg_.velocity[i] =
      get_value(name_if_value_mapping_, joint_names_[i], HW_IF_VELOCITY);
    joint_state_msg_.effort[i] = get_value(name_if_value_mapping_, joint_names_[i], HW_IF_EFFORT);
  }

  for (size_t joint_index = 0; joint_index < dynamic_joint_state_msg_.joint_names.size();
       ++joint_index)
  {
    const auto & name = dynamic_joint_state_msg_.joint_names[joint_index];
    for (size_t interface_index = 0;
         interface_index <
         dynamic_joint_state_msg_.interface_values[joint_index].interface_names.size();
         ++interface_index)
    {
      const auto & interface_name =
        dynamic_joint_state_msg_.interface_values[joint_index].interface_names[interface_index];
      dynamic_joint_state_msg_.interface_values[joint_index].values[interface_index] =
        name_if_value_mapping_[name][interface_name];
    }
  }
  // publish
  joint_state_publisher_->publish(joint_state_msg_);
  dynamic_joint_state_publisher_->publish(dynamic_joint_state_msg_);

  return controller_interface::return_type::OK;
}

}  // namespace joint_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_state_broadcaster::JointStateBroadcaster, controller_interface::ControllerInterface)
