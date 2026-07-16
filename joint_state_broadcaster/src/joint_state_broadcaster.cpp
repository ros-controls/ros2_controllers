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

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstdint>
#include <optional>
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

namespace
{
/// Check if data type is accepted for the measurement time interfaces.
bool is_supported_timestamp_data_type(hardware_interface::HandleDataType data_type)
{
  return data_type == hardware_interface::HandleDataType::DOUBLE ||
         data_type == hardware_interface::HandleDataType::INT32 ||
         data_type == hardware_interface::HandleDataType::UINT32;
}
}  // namespace

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

  if (use_urdf_joint_interfaces())
  {
    state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL_BEST_EFFORT;
    for (const auto & joint : model_.joints_)
    {
      if (
        joint.second->type == urdf::Joint::CONTINUOUS ||
        joint.second->type == urdf::Joint::REVOLUTE || joint.second->type == urdf::Joint::PRISMATIC)
      {
        for (const auto & interface : map_interface_to_joint_state_)
        {
          state_interfaces_config.names.push_back(joint.first + "/" + interface.first);
        }
      }
    }
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

  // Claim the optional measurement time interfaces (if configured).
  if (use_timestamp_interfaces())
  {
    state_interfaces_config.names.push_back(params_.timestamp_state_interfaces.sec);
    state_interfaces_config.names.push_back(params_.timestamp_state_interfaces.nsec);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  const bool timestamp_sec_configured = !params_.timestamp_state_interfaces.sec.empty();
  const bool timestamp_nsec_configured = !params_.timestamp_state_interfaces.nsec.empty();
  if (timestamp_sec_configured != timestamp_nsec_configured)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Both 'timestamp_state_interfaces.sec' and 'timestamp_state_interfaces.nsec' must be set, "
      "or both must be empty.");
    return CallbackReturn::ERROR;
  }

  if (use_urdf_joint_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. Will try to publish all available state "
      "interfaces of the URDF joints.");
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
      // Warn if custom mapping is being ignored
      RCLCPP_WARN_EXPRESSION(
        get_node()->get_logger(), interface != interface_to_map,
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
    if (use_urdf_joint_interfaces())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Robot description could not be loaded. Cannot determine URDF joints to publish. "
        "Either provide a valid robot description, or explicitly set both 'joints' and "
        "'interfaces' parameters.");
      return CallbackReturn::ERROR;
    }
    else
    {
      if (params_.use_urdf_to_filter)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "'use_urdf_to_filter' parameter is set to true, but robot description could not be "
          "parsed. Will publish the joints defined in 'joints' parameter without filtering with "
          "URDF. Fix the robot description to filter joints with URDF.");
      }
      else
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Failed to parse robot description. Will publish the joints defined in 'joints' "
          "parameter along with the defined interfaces in 'interface' parameter.");
      }
    }
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

  // Resolve the optional measurement time interfaces, if configured.
  timestamp_sec_index_.reset();
  timestamp_nsec_index_.reset();
  last_valid_measurement_time_.reset();
  if (use_timestamp_interfaces())
  {
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      const auto name = state_interfaces_[i].get_name();
      if (name == params_.timestamp_state_interfaces.sec)
      {
        timestamp_sec_index_ = i;
      }
      else if (name == params_.timestamp_state_interfaces.nsec)
      {
        timestamp_nsec_index_ = i;
      }
    }
    // If configured but not claimed, or wrong type for the interface,
    // warn and fall back to the controller manager time.
    if (!timestamp_sec_index_.has_value() || !timestamp_nsec_index_.has_value())
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "timestamp_state_interfaces is set (sec='%s', nsec='%s') but the interface(s) were not "
        "found among the claimed state interfaces; header.stamp will use the controller manager "
        "time.",
        params_.timestamp_state_interfaces.sec.c_str(),
        params_.timestamp_state_interfaces.nsec.c_str());
      timestamp_sec_index_.reset();
      timestamp_nsec_index_.reset();
    }
    else if (
      !is_supported_timestamp_data_type(state_interfaces_[*timestamp_sec_index_].get_data_type()) ||
      !is_supported_timestamp_data_type(state_interfaces_[*timestamp_nsec_index_].get_data_type()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "timestamp_state_interfaces have an unsupported data type (sec='%s' is '%s', nsec='%s' is "
        "'%s'); supported types are double, int32 and uint32. header.stamp will use the "
        "controller manager time.",
        params_.timestamp_state_interfaces.sec.c_str(),
        state_interfaces_[*timestamp_sec_index_].get_data_type().to_string().c_str(),
        params_.timestamp_state_interfaces.nsec.c_str(),
        state_interfaces_[*timestamp_nsec_index_].get_data_type().to_string().c_str());
      timestamp_sec_index_.reset();
      timestamp_nsec_index_.reset();
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();
  timestamp_sec_index_.reset();
  timestamp_nsec_index_.reset();
  last_valid_measurement_time_.reset();

  return CallbackReturn::SUCCESS;
}

bool JointStateBroadcaster::init_joint_data()
{
  joint_names_.clear();
  const bool has_joint_state_interface = std::any_of(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [this](const auto & state_interface)
    { return !is_timestamp_interface(state_interface.get_name()); });
  if (!has_joint_state_interface)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint state interfaces found to publish.");
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  const std::vector<std::string> joint_state_interfaces = {
    HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // Skip the measurement time interfaces: they provide header.stamp, not joint state.
    if (is_timestamp_interface(si->get_name()))
    {
      continue;
    }
    if (si->get_data_type() != hardware_interface::HandleDataType::DOUBLE)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "State interface '%s' of joint '%s' has non-double data type and will be ignored.",
        si->get_interface_name().c_str(), si->get_prefix_name().c_str());
      continue;
    }
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
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Interface '%s' of joint '%s' is not mapped to any joint state field. The default value %f "
        "will be used.",
        interface_name.c_str(), prefix_name.c_str(), kUninitializedValue);
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
    else
    {
      // If default interfaces (pos/vel/eff) are missing, log a warning and return NaN in
      // the fields.
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Interface '%s' of joint '%s' is not present in JointState message fields. NaN's will be "
        "filled in the respective field.",
        interface_name.c_str(), prefix_name.c_str());
    }
  }

  std::reverse(joint_names_.begin(), joint_names_.end());
  if (is_model_loaded_ && params_.use_urdf_to_filter && params_.joints.empty())
  {
    // Preserve the order from the first pass; only remove fixed joints
    joint_names_.erase(
      std::remove_if(
        joint_names_.begin(), joint_names_.end(),
        [this](const std::string & name)
        {
          const auto urdf_joint = model_.getJoint(name);
          return !urdf_joint || urdf_joint->type == urdf::Joint::FIXED;
        }),
      joint_names_.end());
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
    // Keep the measurement time interfaces out of the mapping so it stays aligned with the read
    // loop in update(), which skips them too.
    if (is_timestamp_interface(state_interfaces_[i].get_name()))
    {
      continue;
    }
    if (state_interfaces_[i].get_data_type() != hardware_interface::HandleDataType::DOUBLE)
    {
      continue;
    }
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

bool JointStateBroadcaster::use_all_available_interfaces() const
{
  return this->use_urdf_joint_interfaces();
}

bool JointStateBroadcaster::use_urdf_joint_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

bool JointStateBroadcaster::use_timestamp_interfaces() const
{
  return !params_.timestamp_state_interfaces.sec.empty() &&
         !params_.timestamp_state_interfaces.nsec.empty();
}

bool JointStateBroadcaster::is_timestamp_interface(const std::string & full_interface_name) const
{
  return full_interface_name == params_.timestamp_state_interfaces.sec ||
         full_interface_name == params_.timestamp_state_interfaces.nsec;
}

std::optional<int64_t> JointStateBroadcaster::read_time_component(
  std::size_t state_interface_index, int64_t minimum, int64_t maximum) const
{
  const auto & state_interface = state_interfaces_[state_interface_index];

  const auto to_int64 = [minimum, maximum](const auto & opt) -> std::optional<int64_t>
  {
    if (!opt.has_value())
    {
      return std::nullopt;
    }
    const long double value = static_cast<long double>(opt.value());
    if (
      !std::isfinite(value) || value < static_cast<long double>(minimum) ||
      value > static_cast<long double>(maximum) || std::trunc(value) != value)
    {
      return std::nullopt;
    }
    return static_cast<int64_t>(value);
  };

  // Read with the accessor that matches the declared type. A typed get_optional throws on a type
  // mismatch, so dispatch on the data type. Supported types are validated at activation.
  switch (state_interface.get_data_type())
  {
    case hardware_interface::HandleDataType::DOUBLE:
      return to_int64(state_interface.get_optional<double>(0));
    case hardware_interface::HandleDataType::INT32:
      return to_int64(state_interface.get_optional<int32_t>(0));
    case hardware_interface::HandleDataType::UINT32:
      return to_int64(state_interface.get_optional<uint32_t>(0));
    default:
      // Unsupported type, already handled at activation. Fall back to the controller manager time.
      return std::nullopt;
  }
}

std::optional<rclcpp::Time> JointStateBroadcaster::read_measurement_time(
  rcl_clock_type_t clock_type) const
{
  const auto sec =
    read_time_component(*timestamp_sec_index_, 1, std::numeric_limits<int32_t>::max());
  const auto nsec = read_time_component(*timestamp_nsec_index_, 0, 999999999);
  if (!sec.has_value() || !nsec.has_value())
  {
    return std::nullopt;
  }
  return rclcpp::Time(
    static_cast<int32_t>(sec.value()), static_cast<uint32_t>(nsec.value()), clock_type);
}

controller_interface::return_type JointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  size_t map_index = 0u;
  for (auto i = 0u; i < state_interfaces_.size(); ++i)
  {
    // The measurement time interfaces feed header.stamp (below). Skip them here so this
    // stays aligned with the mapping, which excludes them too.
    if (is_timestamp_interface(state_interfaces_[i].get_name()))
    {
      continue;
    }
    if (state_interfaces_[i].get_data_type() == hardware_interface::HandleDataType::DOUBLE)
    {
      // no retries, just try to get the latest value once
      const auto & opt = state_interfaces_[i].get_optional(0);
      if (opt.has_value())
      {
        *mapped_values_[map_index] = opt.value();
      }
      // Always advance map_index for every DOUBLE interface, regardless of whether the read
      // succeeded. If we only advance on success, a temporary read failure (e.g. lock contention
      // on a chained interface) causes all subsequent interfaces to be written into the wrong
      // mapped_values_ slots, corrupting the published joint states.
      ++map_index;
    }
  }

  // Once configured, header.stamp is the measurement time, keeping the last valid value during
  // temporary read failures. A zero stamp means no measurement time is available yet.
  rclcpp::Time stamp = time;
  if (timestamp_sec_index_.has_value() && timestamp_nsec_index_.has_value())
  {
    const auto measurement_time = read_measurement_time(time.get_clock_type());
    if (measurement_time.has_value())
    {
      last_valid_measurement_time_ = measurement_time;
    }
    stamp = last_valid_measurement_time_.value_or(
      rclcpp::Time(static_cast<int64_t>(0), time.get_clock_type()));
  }

  if (realtime_joint_state_publisher_)
  {
    joint_state_msg_.header.stamp = stamp;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_state_msg_.position[i] = joint_states_data_[i].position_;
      joint_state_msg_.velocity[i] = joint_states_data_[i].velocity_;
      joint_state_msg_.effort[i] = joint_states_data_[i].effort_;
    }
    realtime_joint_state_publisher_->try_publish(joint_state_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_state_broadcaster::JointStateBroadcaster, controller_interface::ControllerInterface)
