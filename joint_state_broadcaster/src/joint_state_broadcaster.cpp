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

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
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

controller_interface::CallbackReturn JointStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
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
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces())
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints)
    {
      for (const auto & interface : params_.interfaces)
      {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be published");
    params_.joints.clear();
    params_.interfaces.clear();
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing state interfaces defined in 'joints' and 'interfaces' parameters.");
  }

  auto get_map_interface_parameter =
    [&](std::string const & interface, std::string const & interface_to_map)
  {
    if (
      std::find(params_.interfaces.begin(), params_.interfaces.end(), interface) !=
      params_.interfaces.end())
    {
      map_interface_to_joint_state_[interface] = interface;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Mapping from '%s' to interface '%s' will not be done, because '%s' is defined "
        "in 'interface' parameter.",
        interface_to_map.c_str(), interface.c_str(), interface.c_str());
    }
    else
    {
      map_interface_to_joint_state_[interface_to_map] = interface;
    }
  };

  map_interface_to_joint_state_ = {};
  get_map_interface_parameter(HW_IF_POSITION, params_.map_interface_to_joint_state.position);
  get_map_interface_parameter(HW_IF_VELOCITY, params_.map_interface_to_joint_state.velocity);
  get_map_interface_parameter(HW_IF_EFFORT, params_.map_interface_to_joint_state.effort);

  try
  {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";

    joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      topic_name_prefix + "joint_states", rclcpp::SystemDefaultsQoS());

    realtime_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        joint_state_publisher_);

    dynamic_joint_state_publisher_ =
      get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
        topic_name_prefix + "dynamic_joint_states", rclcpp::SystemDefaultsQoS());

    realtime_dynamic_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(
        dynamic_joint_state_publisher_);
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();

  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description. Will publish all the interfaces with '%s', '%s' and '%s'",
      HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT);
  }

  // joint_names reserve space for all joints
  const auto max_joints_size =
    (params_.joints.empty() ? model_.joints_.size() : params_.joints.size()) +
    params_.extra_joints.size();
  joint_names_.reserve(max_joints_size);
  joint_state_msg_.name.reserve(max_joints_size);
  joint_state_msg_.position.reserve(max_joints_size);
  joint_state_msg_.velocity.reserve(max_joints_size);
  joint_state_msg_.effort.reserve(max_joints_size);

  frame_id_ = params_.frame_id;
  if (frame_id_.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Frame ID is not set.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error initializing joint data. JointStateBroadcaster will not run.");
    return CallbackReturn::ERROR;
  }

  init_auxiliary_data();
  init_joint_state_msg();
  init_dynamic_joint_state_msg();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();

  return CallbackReturn::SUCCESS;
}

bool JointStateBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces found to publish.");
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  const std::vector<std::string> joint_state_interfaces = {
    HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    const std::string prefix_name = si->get_prefix_name();
    // initialize map if name is new
    if (name_if_value_mapping_.count(prefix_name) == 0)
    {
      name_if_value_mapping_[prefix_name] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[prefix_name][interface_name] = kUninitializedValue;

    // filter state interfaces that have at least one of the joint_states fields,
    // the rest will be ignored for this message
    if (
      std::find(joint_state_interfaces.begin(), joint_state_interfaces.end(), interface_name) !=
      joint_state_interfaces.end())
    {
      if (
        !params_.use_urdf_to_filter || !params_.joints.empty() || !is_model_loaded_ ||
        model_.getJoint(prefix_name))
      {
        if (std::find(joint_names_.begin(), joint_names_.end(), prefix_name) == joint_names_.end())
        {
          joint_names_.push_back(prefix_name);
        }
      }
    }
  }
  std::reverse(joint_names_.begin(), joint_names_.end());
  if (is_model_loaded_ && params_.use_urdf_to_filter && params_.joints.empty())
  {
    std::vector<std::string> joint_names_filtered;
    for (const auto & [joint_name, urdf_joint] : model_.joints_)
    {
      if (urdf_joint && urdf_joint->type != urdf::Joint::FIXED)
      {
        if (std::find(joint_names_.begin(), joint_names_.end(), joint_name) != joint_names_.end())
        {
          joint_names_filtered.push_back(joint_name);
        }
      }
    }
    joint_names_ = joint_names_filtered;
  }

  // Add extra joints from parameters, each joint will be added to joint_names_ and
  // name_if_value_mapping_ if it is not already there
  for (const auto & extra_joint_name : params_.extra_joints)
  {
    if (name_if_value_mapping_.count(extra_joint_name) == 0)
    {
      name_if_value_mapping_[extra_joint_name] = {
        {HW_IF_POSITION, 0.0}, {HW_IF_VELOCITY, 0.0}, {HW_IF_EFFORT, 0.0}};
      joint_names_.push_back(extra_joint_name);
    }
  }

  return true;
}

void JointStateBroadcaster::init_auxiliary_data()
{
  // save the mapping of state interfaces to joint states
  mapped_values_.clear();
  for (auto i = 0u; i < state_interfaces_.size(); ++i)
  {
    std::string interface_name = state_interfaces_[i].get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    mapped_values_.push_back(
      &name_if_value_mapping_[state_interfaces_[i].get_prefix_name()][interface_name]);
  }
}

void JointStateBroadcaster::init_joint_state_msg()
{
  const size_t num_joints = joint_names_.size();

  /// @note joint_state_msg_ publishes position, velocity and effort for all joints,
  /// with at least one of these interfaces, the rest are omitted from this message

  // default initialization for joint state message
  joint_state_msg_.header.frame_id = frame_id_;
  joint_state_msg_.name = joint_names_;
  joint_state_msg_.position.resize(num_joints, kUninitializedValue);
  joint_state_msg_.velocity.resize(num_joints, kUninitializedValue);
  joint_state_msg_.effort.resize(num_joints, kUninitializedValue);

  // save joint state data
  auto get_address =
    [&](const std::string & joint_name, const std::string & interface_name) -> const double &
  {
    const auto & interfaces_and_values = name_if_value_mapping_.at(joint_name);
    const auto interface_and_value = interfaces_and_values.find(interface_name);
    if (interface_and_value != interfaces_and_values.cend())
    {
      return interface_and_value->second;
    }
    else
    {
      return kUninitializedValue;
    }
  };

  joint_states_data_.clear();
  for (auto i = 0u; i < joint_names_.size(); ++i)
  {
    joint_states_data_.push_back(JointStateData(
      get_address(joint_names_[i], HW_IF_POSITION), get_address(joint_names_[i], HW_IF_VELOCITY),
      get_address(joint_names_[i], HW_IF_EFFORT)));
  }
}

void JointStateBroadcaster::init_dynamic_joint_state_msg()
{
  dynamic_joint_state_msg_.header.frame_id = frame_id_;
  dynamic_joint_state_msg_.joint_names.clear();
  dynamic_joint_state_msg_.interface_values.clear();
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

  // save dynamic joint state data
  dynamic_joint_states_data_.clear();
  for (auto ji = 0u; ji < dynamic_joint_state_msg_.joint_names.size(); ++ji)
  {
    dynamic_joint_states_data_.push_back(std::vector<const double *>());

    const auto & name = dynamic_joint_state_msg_.joint_names[ji];

    for (auto ii = 0u; ii < dynamic_joint_state_msg_.interface_values[ji].interface_names.size();
         ++ii)
    {
      const auto & interface_name =
        dynamic_joint_state_msg_.interface_values[ji].interface_names[ii];
      dynamic_joint_states_data_[ji].push_back(&name_if_value_mapping_[name][interface_name]);
    }
  }
}

bool JointStateBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

controller_interface::return_type JointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (auto i = 0u; i < state_interfaces_.size(); ++i)
  {
    // no retries, just try to get the latest value once
    const auto & opt = state_interfaces_[i].get_optional(0);
    if (opt.has_value())
    {
      *mapped_values_[i] = opt.value();
    }
  }

  if (realtime_joint_state_publisher_)
  {
    joint_state_msg_.header.stamp = time;

    // update joint state message and dynamic joint state message
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_state_msg_.position[i] = joint_states_data_[i].position_;
      joint_state_msg_.velocity[i] = joint_states_data_[i].velocity_;
      joint_state_msg_.effort[i] = joint_states_data_[i].effort_;
    }
    realtime_joint_state_publisher_->try_publish(joint_state_msg_);
  }

  if (realtime_dynamic_joint_state_publisher_)
  {
    dynamic_joint_state_msg_.header.stamp = time;
    for (auto ji = 0u; ji < dynamic_joint_state_msg_.joint_names.size(); ++ji)
    {
      for (auto ii = 0u; ii < dynamic_joint_state_msg_.interface_values[ji].interface_names.size();
           ++ii)
      {
        dynamic_joint_state_msg_.interface_values[ji].values[ii] =
          *dynamic_joint_states_data_[ji][ii];
      }
    }
    realtime_dynamic_joint_state_publisher_->try_publish(dynamic_joint_state_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_state_broadcaster::JointStateBroadcaster, controller_interface::ControllerInterface)
