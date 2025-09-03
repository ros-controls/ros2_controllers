// Copyright (c) 2025, b»robotized Group
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

#include "gpio_controllers/gpio_tool_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/version.h"

namespace
{  // utility

// Changed services history QoS to keep all so we don't lose any client service calls
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 17
rclcpp::QoS qos_services =
  rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
    .reliable()
    .durability_volatile();
#else
static const rmw_qos_profile_t qos_services = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};
#endif

}  // namespace

namespace gpio_tool_controller
{
GpioToolController::GpioToolController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn GpioToolController::on_init()
{
  current_tool_action_.store(ToolAction::IDLE);
  current_tool_transition_.store(GPIOToolTransition::IDLE);
  target_configuration_.store(std::make_shared<std::string>(""));
  current_state_ = "";
  current_configuration_ = "";

  try
  {
    param_listener_ = std::make_shared<gpio_tool_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GpioToolController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  bool all_good = true;

  if (params_.engaged_joints.empty() && params_.configuration_joints.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No joints defined therefore no joint states publisher is created.");
    joint_states_need_publishing_ = false;
  }
  else
  {
    joint_states_need_publishing_ = true;
  }

  auto check_joint_states_sizes = [this](const size_t joint_states_size, const size_t joint_states_values_size, const std::string & param_name) {
    if (joint_states_size != joint_states_values_size)
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Size of '%s' joint values and defined states must be equal. "
        "Joint states values size: %zu, expected size: %zu.",
        param_name.c_str(), joint_states_values_size, joint_states_size);
      return false;
    }
    return true;
  };

  if (!check_joint_states_sizes(
      params_.engaged_joints.size(), params_.disengaged.joint_states.size(), "engaged"))
  {
    all_good = false;
  }

  if (params_.possible_engaged_states.size() != params_.engaged.states.possible_engaged_states_map.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'possible_engaged_states' and 'engaged.states' parameters must be equal. "
      "Possible engaged states size: %zu, engaged states size: %zu.",
      params_.possible_engaged_states.size(), params_.engaged.states.possible_engaged_states_map.size());
      all_good = false;
  }

  for (const auto & [name, info] : params_.engaged.states.possible_engaged_states_map)
  {
    if (!check_joint_states_sizes(
        params_.engaged_joints.size(), info.joint_states.size(), "engaged " + name + " state"))
    {
      all_good = false;
    }
  }

  // Initialize storage of joint state values
  joint_states_values_.resize(
    params_.engaged_joints.size() + params_.configuration_joints.size(),
    std::numeric_limits<double>::quiet_NaN());

  // check sizes of all other parameters
  auto check_parameter_pairs = [this](const std::vector<std::string> & interfaces, const std::vector<double> & values, const std::string & parameter_name) {
    if (interfaces.size() != values.size()) {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Size of parameter's '%s' interfaces and values must be equal.", parameter_name.c_str());
      return false;
    }
    return true;
  };

  if (!check_parameter_pairs(params_.disengaged.set_before_command.interfaces, params_.disengaged.set_before_command.values, "disengaged before command") ||
      !check_parameter_pairs(params_.disengaged.set_before_state.interfaces, params_.disengaged.set_before_state.values, "disengaged before state") ||
      !check_parameter_pairs(params_.disengaged.command.interfaces, params_.disengaged.command.values, "disengaged command") ||
      !check_parameter_pairs(params_.disengaged.state.interfaces, params_.disengaged.state.values, "disengaged state") ||
      !check_parameter_pairs(params_.disengaged.set_after_command.interfaces, params_.disengaged.set_after_command.values, "disengaged after command") ||
      !check_parameter_pairs(params_.disengaged.set_after_state.interfaces, params_.disengaged.set_after_state.values, "disengaged after state"))
  {
    all_good = false;
  }

  if (!check_parameter_pairs(params_.engaged.set_before_command.interfaces, params_.engaged.set_before_command.values, "engaged before command") ||
  !check_parameter_pairs(params_.engaged.set_before_state.interfaces, params_.engaged.set_before_state.values, "engaged before state") ||
  !check_parameter_pairs(params_.engaged.command.interfaces, params_.engaged.command.values, "engaged command"))
  {
    all_good = false;
  }

  for (const auto & [name, info] : params_.engaged.states.possible_engaged_states_map)
  {
    if (!check_parameter_pairs(info.interfaces, info.values, "engaged " + name + " states") ||
    !check_parameter_pairs(info.set_after_command_interfaces, info.set_after_command_values, "engaged " + name + " after command") ||
    !check_parameter_pairs(info.set_after_state_interfaces, info.set_after_state_values, "engaged " + name + " after state"))
    {
      all_good = false;
    }
  }

  if (params_.configurations.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No configurations defined. Configuration control will be disabled.");
    configuration_control_enabled_ = false;
  }
  else
  {
    configuration_control_enabled_ = true;
  }

  if (params_.configurations.size() != params_.configuration_setup.configurations_map.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'configurations' and 'configuration_setup' parameters must be equal. "
      "Configurations size: %zu, configuration joints size: %zu.",
      params_.configurations.size(), params_.configuration_setup.configurations_map.size());
    all_good = false;
  }
  for (const auto & [name, info] : params_.configuration_setup.configurations_map)
  {
    if (!check_joint_states_sizes(
        params_.configuration_joints.size(), info.joint_states.size(), "configuration " + name) ||
        !check_parameter_pairs(info.set_before_command_interfaces, info.set_before_command_values, "configuration " + name + " before commands") ||
        !check_parameter_pairs(info.set_before_state_interfaces, info.set_before_state_values, "configuration " + name + " before states") ||
        !check_parameter_pairs(info.command_interfaces, info.command_values, "configuration " + name + " commands") ||
        !check_parameter_pairs(info.state_interfaces, info.state_values, "configuration " + name + " states") ||
        !check_parameter_pairs(info.set_after_command_interfaces, info.set_after_command_values, "configuration " + name + " after commands") ||
        !check_parameter_pairs(info.set_after_state_interfaces, info.set_after_state_values, "configuration " + name + " after states")
      )
    {
      all_good = false;
    }
  }

  if (params_.tool_specific_sensors.size() != params_.sensors_interfaces.tool_specific_sensors_map.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'tool_specific_sensors' and 'sensors_interfaces' parameters must be equal. "
      "Tool specific sensors size: %zu, sensors interfaces size: %zu.",
      params_.tool_specific_sensors.size(), params_.sensors_interfaces.tool_specific_sensors_map.size());
    all_good = false;
  }
  for (const auto & [name, info] : params_.sensors_interfaces.tool_specific_sensors_map)
  {
    if (info.interface.empty())
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Interfaces for tool specific sensor '%s' cannot be empty.", name.c_str());
      all_good = false;
    }
  }

  if (!all_good)
  {
    RCLCPP_FATAL(
      get_node()->get_logger(), "One or more parameters are not configured correctly. See above messges for details.");
    return CallbackReturn::FAILURE;
  }

  if (!prepare_command_and_state_ios())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(), "Failed to prepare command and state GPIOs. See above messages for details.");
    return CallbackReturn::FAILURE;
  }

  auto result = prepare_publishers_and_services();
  if (result != controller_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GpioToolController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(command_if_ios_.size());
  for (const auto & command_io : command_if_ios_)
  {
    command_interfaces_config.names.push_back(command_io);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration GpioToolController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_if_ios_.size());
  for (const auto & state_io : state_if_ios_)
  {
    state_interfaces_config.names.push_back(state_io);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn GpioToolController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_change_start_ = get_node()->now();
  check_tool_state(state_change_start_, true);
  // TODO(denis): update ros2_control and then enable this in this version the interfaces are not released when controller fails to activate
  // if (current_state_.empty() || current_configuration_.empty())
  // {
  //   RCLCPP_FATAL(get_node()->get_logger(), "Controller can not be started as tool state can not be determined! Make sure the hardware is connected properly and controller's configuration is correct, than try to activate the controller again.");
  //   return controller_interface::CallbackReturn::FAILURE;
  // }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GpioToolController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_states_values_.resize(
    params_.engaged_joints.size() + params_.configuration_joints.size(),
    std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GpioToolController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  switch (current_tool_action_.load())
  {
    case ToolAction::IDLE:
    {
      // do nothing
      state_change_start_ = time;
      check_tool_state(state_change_start_, false);
      break;
    }
    case ToolAction::DISENGAGING:
    {
      handle_tool_state_transition(
        time, disengaged_gpios_, params_.disengaged.name, joint_states_values_, 0, current_state_);
      break;
    }
    case ToolAction::ENGAGING:
    {
      handle_tool_state_transition(
        time, engaged_gpios_, params_.engaged.name, joint_states_values_, 0, current_state_);
      break;
    }
    case ToolAction::RECONFIGURING:
    {
      handle_tool_state_transition(
        time, reconfigure_gpios_, *(target_configuration_.load()), joint_states_values_, params_.engaged_joints.size(), current_configuration_);
      break;
    }
    case ToolAction::CANCELING:
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "%s - CANCELING: Tool action is being canceled, "
        "going to HALTED. Reset the tool using '~/reset_halted' service. After that set sensible state.", current_state_.c_str());
      current_tool_transition_.store(GPIOToolTransition::HALTED);
      check_tool_state(time, true);
      std::vector<double> tmp_vec;
      std::string tmp_str;
      handle_tool_state_transition(
        time, ToolTransitionIOs(), "", tmp_vec, 0,
        tmp_str);  // parameters don't matter as end up processing only the halted state
      break;
    }
    default:
    {
      break;
    }
  }
  publish_topics(time);

  return controller_interface::return_type::OK;
}

bool GpioToolController::set_commands(
  const std::unordered_map<std::string, std::pair<double, size_t>> & commands, const std::string & output_prefix, const uint8_t next_transition)
{
  bool all_successful = true;
  if (!commands.empty())  // only set commands if present
  {
    for (const auto & [name, pair] : commands)
    {
      const auto & [value, index] = pair;
      if (!command_interfaces_[index].set_value(value))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "%s: Failed to set the command state for %s",
          output_prefix.c_str(), command_interfaces_[index].get_name().c_str());
        all_successful = false;
      }
    }
  }

  if (all_successful)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: Transitioning after setting commands to: %d",
      output_prefix.c_str(), next_transition);
    // when canceling we don't continue the transition
    if (current_tool_action_.load() != ToolAction::CANCELING)
    {
      current_tool_transition_.store(next_transition);
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "%s: Error occured when setting commands - see above for details.",
      output_prefix.c_str());
    current_tool_transition_.store(GPIOToolTransition::HALTED);
  }

  return all_successful;
}

bool GpioToolController::check_states(
  const rclcpp::Time & current_time, const std::unordered_map<std::string, std::pair<double, size_t>> & states, const std::string & output_prefix, const uint8_t next_transition, const bool warning_output)
{
  bool all_correct = true;
  if (!states.empty())  // only check the states if they are present
  {
    for (const auto& [name, pair] : states)
    {
      const auto & [value, index] = pair;
      RCLCPP_DEBUG(
        get_node()->get_logger(), "%s: Looking for state interface '%s' on index %zu, and state size is %zu, and state_interface size is %zu",
        output_prefix.c_str(), state_interfaces_.at(index).get_name().c_str(), index, states.size(), state_interfaces_.size());
      const double current_state_value = state_interfaces_.at(index).get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
      if (std::isnan(current_state_value) ||
        abs(current_state_value - value) > params_.tolerance)
      {
        RCLCPP_WARN_EXPRESSION(
          get_node()->get_logger(), warning_output, "%s: State value for %s doesn't match. Is %.2f, expected %.2f",
          output_prefix.c_str(), state_interfaces_.at(index).get_name().c_str(),
          current_state_value, value);
        all_correct = false;
      }
    }
  }

  if (all_correct)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: Transitionning after reaching state to: %d",
      output_prefix.c_str(), next_transition);
    // when canceling we don't continue transition
    if (current_tool_action_.load() != ToolAction::CANCELING)
    {
      current_tool_transition_.store(next_transition);
    }
  }
  else if ((current_time - state_change_start_).seconds() > params_.timeout)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "%s: Tool didin't reached target state within %.2f seconds. Try resetting the tool using '~/reset_halted' service. After that set sensible state.",
      output_prefix.c_str(), params_.timeout);
    current_tool_transition_.store(GPIOToolTransition::HALTED);
  }

  return all_correct;
}

void GpioToolController::check_tool_state_and_switch(
  const rclcpp::Time & current_time, const ToolTransitionIOs & ios, std::vector<double> & joint_states, const size_t joint_states_start_index, const std::string & output_prefix, const uint8_t next_transition, std::string & found_state_name, const bool warning_output)
{
  for (const auto & [state_name, states] : ios.states)
  {
    bool state_exists = states.size() > 0;
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: Checking state '%s' for tool. States _%s_ exist. Used number of state interfaces %zu.",
      output_prefix.c_str(), state_name.c_str(), (state_exists ? "do" : "do not"), states.size());

    if (check_states(current_time, states, output_prefix, next_transition, warning_output))
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "%s: SUCCESS! state '%s' for tool is confirmed!",
        output_prefix.c_str(), state_name.c_str());
      found_state_name = state_name;
      const auto & js_val = ios.states_joint_states.at(state_name);
      if (joint_states_start_index + js_val.size() <= joint_states.size())
      {
        std::copy(
          js_val.begin(),
          js_val.end(),
          joint_states.begin() + joint_states_start_index);
      }
      else
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "%s - CHECK_STATE: Joint states size (%zu) is not enough to copy the values for state '%s' (size: %zu) from starting index %zu.",
          output_prefix.c_str(), joint_states.size(), state_name.c_str(), js_val.size(), joint_states_start_index);
      }
      break;
    }
  }
}

void GpioToolController::handle_tool_state_transition(
  const rclcpp::Time & current_time, const ToolTransitionIOs & ios,
  const std::string & target_state, std::vector<double> & joint_states, const size_t joint_states_start_index, std::string & end_state)
{
  bool finish_transition_to_state = false;
  switch (current_tool_transition_.load())
  {
    case GPIOToolTransition::IDLE:
      // reset time to avoid any time-source related crashing
      state_change_start_ = current_time;
      // do nothing
      break;

    case GPIOToolTransition::HALTED:
      if (reset_halted_.load())
      {
        // check here the state of the tool if you can figure it out. If not - set unknown.
        reset_halted_.store(false);
        finish_transition_to_state = true;
      }
      break;

    case GPIOToolTransition::SET_BEFORE_COMMAND:
      RCLCPP_DEBUG(
        get_node()->get_logger(), "%s - SET_BEFORE_COMMAND: Tool action is being set before command.",
        target_state.c_str());
      if (!transition_time_updated_.load())
      {
        state_change_start_ = current_time;
        transition_time_updated_.store(true);
      }
      set_commands(
        ios.set_before_commands.at(target_state),
        target_state + " - SET_BEFORE_COMMAND", GPIOToolTransition::CHECK_BEFORE_COMMAND);
      break;
    case GPIOToolTransition::CHECK_BEFORE_COMMAND:
      check_states(
        current_time, ios.set_before_states.at(target_state),
        target_state + " - CHECK_BEFORE_COMMAND", GPIOToolTransition::SET_COMMAND);
      break;
    case GPIOToolTransition::SET_COMMAND:
      set_commands(ios.commands.at(target_state), target_state + " - SET_COMMAND", GPIOToolTransition::CHECK_COMMAND);
      break;
    case GPIOToolTransition::CHECK_COMMAND:
      check_tool_state_and_switch(current_time, ios, joint_states, joint_states_start_index, target_state + " - CHECK_COMMAND", GPIOToolTransition::SET_AFTER_COMMAND, end_state);
      break;
    case GPIOToolTransition::SET_AFTER_COMMAND:
      set_commands(
        ios.set_after_commands.at(end_state),
        target_state + " - SET_AFTER_COMMAND", GPIOToolTransition::CHECK_AFTER_COMMAND);
      break;

    case GPIOToolTransition::CHECK_AFTER_COMMAND:
      if (check_states(current_time, ios.set_after_states.at(end_state), target_state + " - CHECK_AFTER_COMMAND", GPIOToolTransition::IDLE))
      {
        finish_transition_to_state = true;
      }
      break;

    default:
      break;
  }

  if (finish_transition_to_state)
  {
    current_tool_action_.store(ToolAction::IDLE);
    current_tool_transition_.store(GPIOToolTransition::IDLE);
    transition_time_updated_.store(false);  // resetting the flag

    RCLCPP_INFO(
      get_node()->get_logger(), "%s: Tool state or configuration change finished!",
      target_state.c_str());
  }
}

bool GpioToolController::prepare_command_and_state_ios()
{
  auto parse_interfaces_from_params = [&](
                                        const std::vector<std::string> & interfaces,
                                        const std::vector<double> & values,
                                        const std::string & param_name,
                                        std::unordered_map<std::string, std::pair<double, size_t>> & ios,
                                        std::unordered_set<std::string> & interface_list
                                        ) -> void
  {
    if (interfaces.size() != values.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Size of interfaces and values parameters for '%s' must be the same. "
        "Interfaces size: %zu, Values size: %zu",
        param_name.c_str(), interfaces.size(), values.size());
      return;
    }

    ios.reserve(interfaces.size());
    interface_list.reserve(interface_list.size() + interfaces.size());

    for (size_t i = 0; i < interfaces.size(); ++i)
    {
      if (!interfaces[i].empty())
      {
        ios[interfaces[i]].first = values[i];
        ios[interfaces[i]].second = -1;  // -1 means not set yet
        interface_list.insert(interfaces[i]);
      }
    }
  };

  // Disengaged IOs
  disengaged_gpios_.possible_states.push_back(params_.disengaged.name);

  disengaged_gpios_.set_before_commands[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.disengaged.set_before_command.interfaces, params_.disengaged.set_before_command.values,
    "disengaged.set_before_command", disengaged_gpios_.set_before_commands[params_.disengaged.name],
    command_if_ios_);

  disengaged_gpios_.set_before_states[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.disengaged.set_before_state.interfaces, params_.disengaged.set_before_state.values, "disengaged.set_before_state", disengaged_gpios_.set_before_states[params_.disengaged.name], state_if_ios_);

  disengaged_gpios_.commands[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.disengaged.command.interfaces, params_.disengaged.command.values, "disengaged.command", disengaged_gpios_.commands[params_.disengaged.name], command_if_ios_);

  disengaged_gpios_.states[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.disengaged.state.interfaces, params_.disengaged.state.values, "disengaged.state", disengaged_gpios_.states[params_.disengaged.name], state_if_ios_);

  disengaged_gpios_.states_joint_states[params_.disengaged.name] =
    params_.disengaged.joint_states;

  disengaged_gpios_.set_after_commands[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  disengaged_gpios_.set_after_states[params_.disengaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();

  parse_interfaces_from_params(
    params_.disengaged.set_after_command.interfaces, params_.disengaged.set_after_command.values,
    "disengaged.set_after_command", disengaged_gpios_.set_after_commands[params_.disengaged.name], command_if_ios_);
  parse_interfaces_from_params(
    params_.disengaged.set_after_state.interfaces, params_.disengaged.set_after_state.values,
    "disengaged.set_after_state", disengaged_gpios_.set_after_states[params_.disengaged.name], state_if_ios_);

  // Engaged IOs
  engaged_gpios_.set_before_commands[params_.engaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.engaged.set_before_command.interfaces, params_.engaged.set_before_command.values,
    "engaged.set_before_command", engaged_gpios_.set_before_commands[params_.engaged.name], command_if_ios_);
  engaged_gpios_.set_before_states[params_.engaged.name] =
    std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.engaged.set_before_state.interfaces, params_.engaged.set_before_state.values,
    "engaged.set_before_state", engaged_gpios_.set_before_states[params_.engaged.name], state_if_ios_);
  engaged_gpios_.commands[params_.engaged.name] = std::unordered_map<std::string, std::pair<double, size_t>>();
  parse_interfaces_from_params(
    params_.engaged.command.interfaces, params_.engaged.command.values, "engaged.command",
    engaged_gpios_.commands[params_.engaged.name], command_if_ios_);

  engaged_gpios_.possible_states = params_.possible_engaged_states;
  for (const auto & state : params_.possible_engaged_states)
  {
    engaged_gpios_.states[state] = std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.engaged.states.possible_engaged_states_map.at(state).interfaces,
      params_.engaged.states.possible_engaged_states_map.at(state).values, "engaged.states." + state,
      engaged_gpios_.states[state], state_if_ios_);

    engaged_gpios_.states_joint_states[state] =
      params_.engaged.states.possible_engaged_states_map.at(state).joint_states;

    engaged_gpios_.set_after_commands[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    engaged_gpios_.set_after_states[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.engaged.states.possible_engaged_states_map.at(state).set_after_command_interfaces,
      params_.engaged.states.possible_engaged_states_map.at(state).set_after_command_values,
      "engaged.set_after_command." + state, engaged_gpios_.set_after_commands[state], command_if_ios_);
    parse_interfaces_from_params(
      params_.engaged.states.possible_engaged_states_map.at(state).set_after_state_interfaces,
      params_.engaged.states.possible_engaged_states_map.at(state).set_after_state_values,
      "engaged.set_after_state." + state, engaged_gpios_.set_after_states[state], state_if_ios_);
  }

  // Configurations IOs
  reconfigure_gpios_.possible_states = params_.configurations;
  for (const auto & state : reconfigure_gpios_.possible_states)
  {
    reconfigure_gpios_.set_before_commands[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).set_before_command_interfaces,
      params_.configuration_setup.configurations_map.at(state).set_before_command_values,
      "reconfigure.set_before_command." + state, reconfigure_gpios_.set_before_commands[state],
      command_if_ios_);

    reconfigure_gpios_.set_before_states[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).set_before_state_interfaces,
      params_.configuration_setup.configurations_map.at(state).set_before_state_values,
      "reconfigure.set_before_state." + state, reconfigure_gpios_.set_before_states[state], state_if_ios_);

    reconfigure_gpios_.commands[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).command_interfaces,
      params_.configuration_setup.configurations_map.at(state).command_values,
      "reconfigure.command." + state, reconfigure_gpios_.commands[state], command_if_ios_);

    reconfigure_gpios_.states[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).state_interfaces,
      params_.configuration_setup.configurations_map.at(state).state_values,
      "reconfigure.state." + state, reconfigure_gpios_.states[state], state_if_ios_);

    reconfigure_gpios_.states_joint_states[state] =
      params_.configuration_setup.configurations_map.at(state).joint_states;

    reconfigure_gpios_.set_after_commands[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).set_after_command_interfaces,
      params_.configuration_setup.configurations_map.at(state).set_after_command_values,
      "reconfigure.set_after_command." + state, reconfigure_gpios_.set_after_commands[state],
      command_if_ios_);
    reconfigure_gpios_.set_after_states[state] =
      std::unordered_map<std::string, std::pair<double, size_t>>();
    parse_interfaces_from_params(
      params_.configuration_setup.configurations_map.at(state).set_after_state_interfaces,
      params_.configuration_setup.configurations_map.at(state).set_after_state_values,
      "reconfigure.set_after_state." + state, reconfigure_gpios_.set_after_states[state], state_if_ios_);
  }

  for (const auto & [name, data] : params_.sensors_interfaces.tool_specific_sensors_map)
  {
    state_if_ios_.insert(data.interface);
  }

  auto store_indices_of_interfaces = [&](std::unordered_map<std::string, std::pair<double, size_t>> & ios_map, const std::unordered_set<std::string> & interfaces) -> bool {
    for (auto & [itf_name, pair] : ios_map)
    {
      auto & [value, index] = pair;
      auto it = std::find(interfaces.begin(), interfaces.end(), itf_name);
      if (it != interfaces.end())
      {
        index = std::distance(interfaces.begin(), it);
      }
      else
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Interface '%s' not found in the list of interfaces.", itf_name.c_str());
        return false;
      }
    }
    return true;
  };

  bool ret = true;

  // Disengaged IOs
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.set_before_commands[params_.disengaged.name], command_if_ios_);
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.set_before_states[params_.disengaged.name], state_if_ios_);
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.commands[params_.disengaged.name], command_if_ios_);
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.states[params_.disengaged.name], state_if_ios_);
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.set_after_commands[params_.disengaged.name], command_if_ios_);
  ret &= store_indices_of_interfaces(
    disengaged_gpios_.set_after_states[params_.disengaged.name], state_if_ios_);

  // Engaged IOs
  ret &= store_indices_of_interfaces(
    engaged_gpios_.set_before_commands[params_.engaged.name], command_if_ios_);
  ret &= store_indices_of_interfaces(
    engaged_gpios_.set_before_states[params_.engaged.name], state_if_ios_);
  ret &= store_indices_of_interfaces(
    engaged_gpios_.commands[params_.engaged.name], command_if_ios_);
  for (const auto & state : params_.possible_engaged_states)
  {
    ret &= store_indices_of_interfaces(engaged_gpios_.states[state], state_if_ios_);
    ret &= store_indices_of_interfaces(
      engaged_gpios_.set_after_commands[state], command_if_ios_);
    ret &= store_indices_of_interfaces(
      engaged_gpios_.set_after_states[state], state_if_ios_);
  }

  // Reconfigure IOs
  for (const auto & state : reconfigure_gpios_.possible_states)
  {
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.set_before_commands[state], command_if_ios_);
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.set_before_states[state], state_if_ios_);
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.commands[state], command_if_ios_);
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.states[state], state_if_ios_);
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.set_after_commands[state], command_if_ios_);
    ret &= store_indices_of_interfaces(
      reconfigure_gpios_.set_after_states[state], state_if_ios_);
  }

  return ret;
}

GpioToolController::EngagingSrvType::Response GpioToolController::process_engaging_request(
  const ToolAction & requested_action, const std::string & requested_action_name)
{
  EngagingSrvType::Response response;
  if (current_tool_action_.load() == ToolAction::RECONFIGURING)
  {
    response.success = false;
    response.message = "Cannot engage the Tool while reconfiguring";
    RCLCPP_ERROR(get_node()->get_logger(), "%s", response.message.c_str());
    return response;
  }

  if (current_tool_action_.load() != ToolAction::IDLE)
  {
    const auto & current_action_name =
      (current_tool_action_.load() == ToolAction::ENGAGING) ? params_.engaged.name : params_.disengaged.name;

    if (current_tool_action_.load() != requested_action)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Stopping tool '%s' and starting '%s'.",
      current_action_name.c_str(), requested_action_name.c_str());
    }
    else
    {
      response.success = false;
      response.message = "Tool is already executing action '" + requested_action_name + "'. Nothing to do.";
      RCLCPP_INFO(get_node()->get_logger(), "%s", response.message.c_str());
      return response;
    }
  }
  current_tool_action_.store(requested_action);
  current_tool_transition_.store(GPIOToolTransition::SET_BEFORE_COMMAND);

  response.success = true;
  response.message = "Tool action '" + requested_action_name + "' started.";
  return response;
}

GpioToolController::EngagingSrvType::Response GpioToolController::process_reconfigure_request(
  const std::string & config_name)
{
  EngagingSrvType::Response response;
  response.success = true;
  if (config_name.empty())
  {
    response.success = false;
    response.message = "Configuration name cannot be empty";
  }
  if (response.success && std::find(params_.configurations.begin(), params_.configurations.end(), config_name) == params_.configurations.end())
  {
    response.success = false;
    response.message = "Configuration '" + config_name + "' does not exist";
  }
  if (response.success && current_tool_action_.load() != ToolAction::IDLE)
  {
    response.success = false;
    response.message = "Tool is currently reconfiguring or executing '"
                        + params_.engaged.name + "' or '" + params_.disengaged.name
                        + "' action.Please wait until it finishes.";
  }
  // This is OK to access `current_state_` as we are in the IDLE state and it is not being modified
  if (response.success && current_state_ != params_.disengaged.name)
  {
    response.success = false;
    response.message = "Tool can be reconfigured only in '" + params_.disengaged.name + "' state. Current state is '" + current_state_ + "'.";
  }
  if (response.success)
  {
    current_tool_action_.store(ToolAction::RECONFIGURING);
    current_tool_transition_.store(GPIOToolTransition::SET_BEFORE_COMMAND);
    target_configuration_.store(std::make_shared<std::string>(config_name));
    response.message = "Tool reconfiguration to '" + config_name + "' has started.";
    RCLCPP_INFO(get_node()->get_logger(), "%s", response.message.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", response.message.c_str());
  }

  return response;
}

GpioToolController::EngagingSrvType::Response GpioToolController::service_wait_for_transition_end(
  const std::string & requested_action_name)
{
  EngagingSrvType::Response response;
  while (current_tool_action_.load() != ToolAction::IDLE)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (current_tool_transition_.load() == GPIOToolTransition::HALTED)
    {
      response.success = false;
      response.message = "Tool action or reconfiguration '" + requested_action_name + "' was halted. Reset it with '~/reset_halted' service.";
      return response;
    }
  }
  response.success = true;
  response.message = "Tool action or reconfiguration '" + requested_action_name + "' completed successfully.";
  return response;
}

controller_interface::CallbackReturn GpioToolController::prepare_publishers_and_services()
{
  if (!params_.use_action)
  {
    // callback groups for each service
    disengaging_service_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    engaging_service_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (configuration_control_enabled_)
    {
      reconfigure_service_callback_group_ =
        get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    auto disengaged_service_callback = [&](
                                   const std::shared_ptr<EngagingSrvType::Request> /*request*/,
                                   std::shared_ptr<EngagingSrvType::Response> response)
    {
      auto result = process_engaging_request(ToolAction::DISENGAGING, params_.disengaged.name);
      if (result.success)
      {
        result = service_wait_for_transition_end(params_.disengaged.name);
      }
      response->success = result.success;
      response->message = result.message;
    };

    disengaged_service_ = get_node()->create_service<EngagingSrvType>(
      "~/" + params_.disengaged.name, disengaged_service_callback, qos_services, disengaging_service_callback_group_);

    auto engaged_service_callback = [&](
                                    const std::shared_ptr<EngagingSrvType::Request> /*request*/,
                                    std::shared_ptr<EngagingSrvType::Response> response)
    {
      auto result = process_engaging_request(ToolAction::ENGAGING, params_.engaged.name);
      if (result.success)
      {
        result = service_wait_for_transition_end(params_.engaged.name);
      }
      response->success = result.success;
      response->message = result.message;
    };

    engaged_service_ = get_node()->create_service<EngagingSrvType>(
      "~/" + params_.engaged.name, engaged_service_callback, qos_services, engaging_service_callback_group_);

    if (configuration_control_enabled_)
    {
      // configure tool service
      auto reconfigure_tool_service_callback =
        [&](
          const std::shared_ptr<ConfigSrvType::Request> request,
          std::shared_ptr<ConfigSrvType::Response> response)
      {
        auto result = process_reconfigure_request(request->config_name);

        if (result.success)
        {
          result = service_wait_for_transition_end(request->config_name);

        }
        response->success = result.success;
        response->message = result.message;
      };
      reconfigure_tool_service_ = get_node()->create_service<ConfigSrvType>(
        "~/reconfigure", reconfigure_tool_service_callback, qos_services,
        reconfigure_service_callback_group_);
    }
  }
  else
  {
    // disengaged close action server
    engaging_action_server_ = rclcpp_action::create_server<EngagingActionType>(
      get_node(), "~/" + params_.engaged.name + "_" + params_.disengaged.name,
      std::bind(
        &GpioToolController::handle_engaging_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GpioToolController::handle_engaging_cancel, this, std::placeholders::_1),
      std::bind(&GpioToolController::handle_action_accepted<EngagingActionType>, this, std::placeholders::_1));

    if (configuration_control_enabled_)
    {
      // reconfigure action server
      config_action_server_ = rclcpp_action::create_server<ConfigActionType>(
        get_node(), "~/reconfigure",
        std::bind(
          &GpioToolController::handle_config_goal, this, std::placeholders::_1,
          std::placeholders::_2),
        std::bind(&GpioToolController::handle_config_cancel, this, std::placeholders::_1),
        std::bind(&GpioToolController::handle_action_accepted<ConfigActionType>, this, std::placeholders::_1));
    }
  }

  reset_service_callback_group_ =
      get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto reset_callback = [&](
                          const std::shared_ptr<ResetSrvType::Request> /*request*/,
                          std::shared_ptr<ResetSrvType::Response> response)
  {
    reset_halted_.store(true);
    response->success = true;
  };
  reset_service_ = get_node()->create_service<ResetSrvType>(
    "~/reset_halted", reset_callback, qos_services, reset_service_callback_group_);

  try
  {
    if (joint_states_need_publishing_)
    {
      t_js_publisher_ =
        get_node()->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SystemDefaultsQoS());
      tool_joint_state_publisher_ = std::make_unique<ToolJointStatePublisher>(t_js_publisher_);
    }

    if_publisher_ = get_node()->create_publisher<DynInterfaceMsg>(
      "~/dynamic_interfaces", rclcpp::SystemDefaultsQoS());
    interface_publisher_ = std::make_unique<InterfacePublisher>(if_publisher_);

    t_s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ = std::make_unique<ControllerStatePublisher>(t_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.engaged.name.empty() || !params_.disengaged.name.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No joints defined, so no joint states will be published, althrough the publisher is "
      "initialized.");
  }

  // if (joint_states_values_.size() == 0)
  // {
  //   RCLCPP_DEBUG(
  //     get_node()->get_logger(),
  //     "Joint states values are empty, resizing to match the number (%zu) of engaged and configuration joints. This should not happen here, as the joint states should be set in the configure method.",
  //     params_.engaged_joints.size() + params_.configuration_joints.size());
  //   joint_states_values_.resize(
  //     params_.engaged_joints.size() + params_.configuration_joints.size(),
  //     std::numeric_limits<double>::quiet_NaN());
  // }

  if (joint_states_need_publishing_)
  {
    tool_joint_state_publisher_->msg_.name.reserve(joint_states_values_.size());
    tool_joint_state_publisher_->msg_.position = joint_states_values_;

    for (const auto & joint_name : params_.engaged_joints)
    {
      tool_joint_state_publisher_->msg_.name.push_back(joint_name);
    }
    for (const auto & joint_name : params_.configuration_joints)
    {
      tool_joint_state_publisher_->msg_.name.push_back(joint_name);
    }
  }

  interface_publisher_->msg_.states.interface_names.reserve(state_if_ios_.size());
  interface_publisher_->msg_.states.values.resize(state_if_ios_.size());
  interface_publisher_->msg_.commands.interface_names.reserve(command_if_ios_.size());
  interface_publisher_->msg_.commands.values.resize(command_if_ios_.size());
  for (const auto & state_io : state_if_ios_)
  {
    interface_publisher_->msg_.states.interface_names.push_back(state_io);
  }
  for (const auto & command_io : command_if_ios_)
  {
    interface_publisher_->msg_.commands.interface_names.push_back(command_io);
  }

  controller_state_publisher_->msg_.state = current_state_;
  controller_state_publisher_->msg_.configuration = current_configuration_;
  controller_state_publisher_->msg_.current_transition.state = current_tool_transition_.load();

  return controller_interface::CallbackReturn::SUCCESS;
}

void GpioToolController::publish_topics(const rclcpp::Time & time)
{
  if (joint_states_need_publishing_)
  {
    if (tool_joint_state_publisher_ && tool_joint_state_publisher_->trylock())
    {
      tool_joint_state_publisher_->msg_.header.stamp = time;
      tool_joint_state_publisher_->msg_.position = joint_states_values_;
    }
    tool_joint_state_publisher_->unlockAndPublish();
  }

  if (interface_publisher_ && interface_publisher_->trylock())
  {
    interface_publisher_->msg_.header.stamp = time;
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      interface_publisher_->msg_.states.values.at(i) =
      static_cast<float>(state_interfaces_.at(i).get_optional<double>().value_or(
        std::numeric_limits<double>::quiet_NaN()));
      }
      for (size_t i = 0; i < command_interfaces_.size(); ++i)
      {
        interface_publisher_->msg_.commands.values.at(i) =
          static_cast<float>(command_interfaces_.at(i).get_optional<double>().value_or(
            std::numeric_limits<double>::quiet_NaN()));
      }
    interface_publisher_->unlockAndPublish();
  }
  if (controller_state_publisher_ && controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.state = current_state_;
    controller_state_publisher_->msg_.configuration = current_configuration_;
    controller_state_publisher_->msg_.current_transition.state = current_tool_transition_.load();
    controller_state_publisher_->unlockAndPublish();
  }
}

rclcpp_action::GoalResponse GpioToolController::handle_engaging_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const EngagingActionType::Goal> goal)
{
  auto engaging_action = ToolAction::DISENGAGING;
  std::string action_name = params_.disengaged.name;
  if (goal->engage)
  {
    engaging_action = ToolAction::ENGAGING;
    action_name = params_.engaged.name;
  }

  auto result = process_engaging_request(engaging_action, action_name);

  if (!result.success)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to process '%s_%s' request: %s", params_.engaged.name.c_str(), params_.disengaged.name.c_str(), result.message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GpioToolController::handle_engaging_cancel(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<EngagingActionType>> /*goal_handle*/)
{
  RCLCPP_WARN(
    get_node()->get_logger(),
    "Tool action is being canceled, going to HALTED state. If you want to reset the Tool, use '~/reset_halted' service.");
  current_tool_action_.store(ToolAction::CANCELING);
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GpioToolController::handle_config_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ConfigActionType::Goal> goal)
{
  auto result = process_reconfigure_request(goal->config_name);
  if (!result.success)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to process reconfigure request: %s", result.message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GpioToolController::handle_config_cancel(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ConfigActionType>> /*goal_handle*/)
{
  RCLCPP_WARN(
    get_node()->get_logger(),
    "Tool action is being canceled, going to HALTED state. If you want to reset the Tool, use '~/reset_halted' service.");
  current_tool_action_.store(ToolAction::CANCELING);
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
void GpioToolController::handle_action_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
{
  auto result = std::make_shared<typename ActionT::Result>();
  auto feedback = std::make_shared<typename ActionT::Feedback>();

  while (true)
  {
    if (current_tool_action_.load() == ToolAction::IDLE)
    {
      result->success = true;
      result->resulting_state_name = current_state_;
      result->message = "Tool action successfully executed!";
      goal_handle->succeed(result);
      break;
    }
    else if (current_tool_transition_.load() == GPIOToolTransition::HALTED)
    {
      result->success = false;
      result->resulting_state_name = current_state_;
      result->message = "Tool action canceled or halted! Check the error, reset the tool using '~/reset_halted' service and set to sensible state.";
      goal_handle->abort(result);
      break;
    }
    else
    {
      feedback->transition.state = current_tool_transition_.load();
      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void GpioToolController::check_tool_state(const rclcpp::Time & current_time, const bool warning_output)
{
  current_state_ = "";
  check_tool_state_and_switch(current_time, disengaged_gpios_, joint_states_values_, 0, params_.disengaged.name + " - CHECK STATES", GPIOToolTransition::IDLE, current_state_, warning_output);
  if (current_state_.empty())
  {
    check_tool_state_and_switch(current_time, engaged_gpios_, joint_states_values_, 0, params_.engaged.name + " - CHECK STATES", GPIOToolTransition::IDLE, current_state_, warning_output);
  }
  if (current_state_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Tool state can not be determined, triggering CANCELING action and HALTED transition.");
    current_tool_action_.store(ToolAction::CANCELING);
  }

  if (configuration_control_enabled_)
  {
    current_configuration_ = "";
    check_tool_state_and_switch(current_time, reconfigure_gpios_, joint_states_values_, params_.engaged_joints.size(), "reconfigure - CHECK STATES", GPIOToolTransition::IDLE, current_configuration_, warning_output);
    if (current_configuration_.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Tool configuration can not be determined, triggering CANCELING action and HALTED transition.");
        current_tool_action_.store(ToolAction::CANCELING);
    }
  }
}

}  // namespace gpio_tool_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gpio_tool_controller::GpioToolController, controller_interface::ControllerInterface)
