// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "io_gripper_controller/io_gripper_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

}  // namespace

namespace io_gripper_controller
{
IOGripperController::IOGripperController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn IOGripperController::on_init()
{
  service_buffer_.initRT(service_mode_type::IDLE);
  configuration_key_ = "";
  configure_gripper_buffer_.initRT(configuration_key_);
  gripper_state_buffer_.initRT(gripper_state_type::IDLE);
  reconfigure_state_buffer_.initRT(reconfigure_state_type::IDLE);

  try
  {
    param_listener_ = std::make_shared<io_gripper_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOGripperController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  auto result = check_parameters();
  if (result != controller_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }

  prepare_command_and_state_ios();

  result = prepare_publishers_and_services();
  if (result != controller_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IOGripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(command_if_ios.size());
  for (const auto & command_io : command_if_ios)
  {
    command_interfaces_config.names.push_back(command_io);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration IOGripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_if_ios.size());
  for (const auto & state_io : state_if_ios)
  {
    state_interfaces_config.names.push_back(state_io);
  }
  
  return state_interfaces_config;
}

controller_interface::CallbackReturn IOGripperController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  check_gripper_and_reconfigure_state();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOGripperController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_state_values_.resize(params_.open_close_joints.size() + params_.configuration_joints.size(), std::numeric_limits<double>::quiet_NaN()); 
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IOGripperController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (reconfigureFlag_.load())
  {
    configuration_key_ = *(configure_gripper_buffer_.readFromRT());
    handle_reconfigure_state_transition(*(reconfigure_state_buffer_.readFromRT()));
  }

  switch (*(service_buffer_.readFromRT()))
  {
  case service_mode_type::IDLE:
    // do nothing
    break;
  case service_mode_type::OPEN:
    handle_gripper_state_transition_open(*(gripper_state_buffer_.readFromRT()));
    break;
  case service_mode_type::CLOSE:
    handle_gripper_state_transition_close(*(gripper_state_buffer_.readFromRT()));
    break;
  
  default:
    break;
  }
  
  publish_gripper_joint_states();
  publish_dynamic_interface_values();

  return controller_interface::return_type::OK;
}

bool IOGripperController::find_and_set_command(const std::string & name, const double value)
{
  auto it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const hardware_interface::LoanedCommandInterface & command_interface)
    {
      return command_interface.get_name() == name;
    });

  if (it != command_interfaces_.end())
  {
    it->set_value(value);
    return true;
  }
  return false;
}

bool IOGripperController::find_and_get_state(const std::string & name, double& value)
{
  auto it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&](const hardware_interface::LoanedStateInterface & state_interface)
    {
      return state_interface.get_name() == name;
    });

  if (it != state_interfaces_.end())
  {
    value = it->get_value();
    return true;
  }
  value = 0.0f;
  return false;
}

bool IOGripperController::find_and_get_command(const std::string & name, double& value)
{
  auto it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const hardware_interface::LoanedCommandInterface & command_interface)
    {
      return command_interface.get_name() == name;
    });

  if (it != command_interfaces_.end())
  {
    value = it->get_value();
    return true;
  }
  value = 0.0f;
  return false;
}

void IOGripperController::handle_gripper_state_transition_close(const gripper_state_type & state)
{
      switch (state)
      {
      case gripper_state_type::IDLE:
        // do nothing
        break;
      case gripper_state_type::SET_BEFORE_COMMAND:
        for (size_t i = 0; i < set_before_command_close.size(); ++i)
        {
          setResult = find_and_set_command(set_before_command_close[i], set_before_command_close_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_before_command_close[i].c_str());
          }
        }

        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CLOSE_GRIPPER);
        break;
      case gripper_state_type::CLOSE_GRIPPER:
        for (size_t i = 0; i < command_ios_close.size(); ++i)
        {
          setResult = find_and_set_command(command_ios_close[i], command_ios_close_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", command_ios_close[i].c_str());
          }
        }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CHECK_GRIPPER_STATE);
        break;
      case gripper_state_type::CHECK_GRIPPER_STATE:
        for (const auto & [state_name, state_params] : params_.close.state.possible_closed_states_map)
        {
          check_state_ios_ = false;
          for (const auto & high_val : state_params.high)
          {
            setResult = find_and_get_state(high_val, state_value_);
            if (!setResult)
            {
              RCLCPP_ERROR(
                get_node()->get_logger(), "Failed to get the state for %s", high_val.c_str());
            }
            else {
              if (abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon())
              {
                check_state_ios_ = true;
              }
              else {
                check_state_ios_ = false;
                break;
              }
            }
          }
          for (const auto & low_val : state_params.low)
          {
            setResult = find_and_get_state(low_val, state_value_);
            if (!setResult)
            {
              RCLCPP_ERROR(
                get_node()->get_logger(), "Failed to get the state for %s", low_val.c_str());
            }
            else {
              if (abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon())
              {
                check_state_ios_ = true;
              }
              else {
                check_state_ios_ = false;
                break;
              }
            }
          }
          if (check_state_ios_)
          {
            closed_state_name_ = state_name; 
            gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_AFTER_COMMAND);
            break;
          }
        }
        break;
      case gripper_state_type::SET_AFTER_COMMAND:
        closed_state_values_ = params_.close.state.possible_closed_states_map.at(closed_state_name_);

        for (const auto & high_val : closed_state_values_.set_after_command_high)
        {
          setResult = find_and_set_command(high_val, 1.0);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", high_val.c_str());
          }
        }

        for (const auto & low_val : closed_state_values_.set_after_command_low)
        {
          setResult = find_and_set_command(low_val, 0.0);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", low_val.c_str());
          }
        }
        for (size_t i = 0; i < params_.close.state.possible_closed_states_map.at(closed_state_name_).joint_states.size(); ++i)
        {
          joint_state_values_[i] = params_.close.state.possible_closed_states_map.at(closed_state_name_).joint_states[i];
        }
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::IDLE);
        closeFlag_.store(false);
        service_buffer_.writeFromNonRT(service_mode_type::IDLE);
        break;
      default:
        break;
      }
}

void IOGripperController::handle_gripper_state_transition_open(const gripper_state_type & state)
{
      switch (state)
      {
      case gripper_state_type::IDLE:
        // do nothing
        break;
      case gripper_state_type::SET_BEFORE_COMMAND:
        for (size_t i = 0; i < set_before_command_open.size(); ++i)
        {
          setResult = find_and_set_command(set_before_command_open[i], set_before_command_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_before_command_open[i].c_str());
          }
        }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::OPEN_GRIPPER); 
        break;
      case gripper_state_type::OPEN_GRIPPER:
        // now open the gripper
        for (size_t i = 0; i < command_ios_open.size(); ++i)
        {
          setResult = find_and_set_command(command_ios_open[i], command_ios_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", command_ios_open[i].c_str());
          }
        }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CHECK_GRIPPER_STATE);
        break;
      case gripper_state_type::CHECK_GRIPPER_STATE:
        // check the state of the gripper
        check_state_ios_ = false;
        for (size_t i = 0; i < state_ios_open.size(); ++i)
        {
          setResult = find_and_get_state(state_ios_open[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", state_ios_open[i].c_str());
          }
          else {
            if (abs(state_value_ - state_ios_open_values[i]) < std::numeric_limits<double>::epsilon())
            {
              check_state_ios_ = true;
            }
            else {
              check_state_ios_ = false;
              break;
            }
          }
        }
        if(check_state_ios_)
        {
          gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_AFTER_COMMAND);
        }
        break;
      case gripper_state_type::SET_AFTER_COMMAND:
        for (size_t i = 0; i < set_after_command_open.size(); ++i)
        {
          setResult = find_and_set_command(set_after_command_open[i], set_after_command_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_after_command_open[i].c_str());
          }
        }
        for (size_t i = 0; i < params_.open.joint_states.size(); ++i)
        {
          joint_state_values_[i] = params_.open.joint_states[i];
        }
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::IDLE);
        openFlag_.store(false);
        service_buffer_.writeFromNonRT(service_mode_type::IDLE);
        break;
      default:
        break;
      }
}

void IOGripperController::handle_reconfigure_state_transition(const reconfigure_state_type & state)
{
  switch (state)
    {
    case reconfigure_state_type::IDLE:
      // do nothing
      break;
    case reconfigure_state_type::FIND_CONFIG:
      config_index_ = std::find(configurations_list_.begin(), configurations_list_.end(), configuration_key_);
      if (config_index_ == configurations_list_.end())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Configuration not found");
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::IDLE);
      }
      else
      {
        conf_it_ = config_map_[std::distance(configurations_list_.begin(), config_index_)];
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::SET_COMMAND);
      }
      break;

    case reconfigure_state_type::SET_COMMAND:
      setResult = false;
      for (const auto & io : conf_it_.command_high)
      {
        setResult = find_and_set_command(io, 1.0);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", io.c_str());
        }
      }
      for (const auto & io : conf_it_.command_low)
      {
        setResult = find_and_set_command(io, 0.0);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", io.c_str());
        }
      }
      reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::CHECK_STATE);
      break;

    case reconfigure_state_type::CHECK_STATE:
      check_state_ios_ = false;
      for (const auto & io : conf_it_.state_high)
      {
        setResult = find_and_get_state(io, state_value_);
        if (!setResult)
        {
          check_state_ios_ = false;
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else 
        {
            if (!(std::abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon()))
          {
            check_state_ios_ = false;
            RCLCPP_ERROR(
              get_node()->get_logger(), "value for state doesn't match %s", io.c_str());
            break;
          }
          else {
            check_state_ios_ = true;
          }
        }
      }

      for (const auto & io : conf_it_.state_low)
      {
        setResult = find_and_get_state(io, state_value_);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else
        { 
          if (!(std::abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon()))
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "value doesn't match %s", io.c_str());
            check_state_ios_ = false;
            break;
          }
          else 
          {
            check_state_ios_ = true;
          }
        }
      }

      if (check_state_ios_)
      {
        for (size_t i = 0; i < conf_it_.joint_states.size(); ++i)
        {
            joint_state_values_[i + params_.open_close_joints.size()] = conf_it_.joint_states[i];
        }
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::IDLE);
        configure_gripper_buffer_.writeFromNonRT("");
        reconfigureFlag_.store(false);
      }
      break;
    default:
      break;
    }
}

controller_interface::CallbackReturn IOGripperController::check_parameters()
{
  if (params_.open_close_joints.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open close joints parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.open.joint_states.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of joint states parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.open.command.high.empty() and params_.open.command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.open.state.high.empty() and params_.open.state.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open state high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.close.set_before_command.high.empty() and params_.close.set_before_command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of set before command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.close.command.high.empty() and params_.close.command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of close command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // configurations parameter
  if (params_.configurations.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configurations parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // configuration joints parameter
  if (params_.configuration_joints.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configuration joints parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // configuration setup parameter
  if (params_.configuration_setup.configurations_map.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configuration map parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // gripper_specific_sensors parameter
  if (params_.gripper_specific_sensors.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of gripper specific sensors parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // sensors interfaces parameter
  if (params_.sensors_interfaces.gripper_specific_sensors_map.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of sensors interfaces parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // if sensor input string is empty then return failure
  for (const auto& [key, val] : params_.sensors_interfaces.gripper_specific_sensors_map)
  {
    if (val.input == "")
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Size of sensor input string parameter cannot be empty ("").");
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}

void IOGripperController::prepare_command_and_state_ios()
{
  // make full command ios lists -- just once
  for (const auto& key : params_.open.set_before_command.high) {
    if(!key.empty())
    {
      set_before_command_open.push_back(key);
      set_before_command_open_values.push_back(1.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.open.set_before_command.low) {
    if(!key.empty())
    {
      set_before_command_open.push_back(key);
      set_before_command_open_values.push_back(0.0);
      command_if_ios.insert(key);
    }
  } 
  for (const auto& key : params_.open.command.high) {
    if (!key.empty())
    {
      command_ios_open.push_back(key);
      command_ios_open_values.push_back(1.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.open.command.low) {
    if (!key.empty())
    {
      command_ios_open.push_back(key);
      command_ios_open_values.push_back(0.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.open.set_after_command.high) {
    if(!key.empty())
    {
      set_after_command_open.push_back(key);
      set_after_command_open_values.push_back(1.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.open.set_after_command.low) {
    if(!key.empty())
    {
      set_after_command_open.push_back(key);
      set_after_command_open_values.push_back(0.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.close.set_before_command.high) {
    if(!key.empty())
    {
      set_before_command_close.push_back(key);
      set_before_command_close_values.push_back(1.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.close.set_before_command.low) {
    if(!key.empty())
    {
      set_before_command_close.push_back(key);
      set_before_command_close_values.push_back(0.0);
      command_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.close.command.high) {
    command_ios_close.push_back(key);
    command_ios_close_values.push_back(1.0);
    command_if_ios.insert(key);
  }

  for (const auto& key : params_.close.command.low) {
    if(!key.empty())
    {
      command_ios_close.push_back(key);
      command_ios_close_values.push_back(0.0);
      command_if_ios.insert(key);
    }
  }

  // make full state ios lists -- just once
  for (const auto& key : params_.open.state.high) {
    if(!key.empty())
    {
      state_ios_open.push_back(key);
      state_ios_open_values.push_back(1.0);
      state_if_ios.insert(key);
    }
  }

  for (const auto& key : params_.open.state.low) {
    if(!key.empty())
    {
      state_ios_open.push_back(key);
      state_ios_open_values.push_back(0.0);
      state_if_ios.insert(key);
    }
  }

  for (const auto & [name, value] : params_.close.state.possible_closed_states_map)
  {
    for (const auto & key : value.high)
    {
      if(!key.empty())
      {
        state_if_ios.insert(key);
      }
    }
    for (const auto & key : value.low)
    {
      if(!key.empty())
      {
        state_if_ios.insert(key);
      }
    }
    for (const auto & key: value.set_after_command_high)
    {
      if(!key.empty())
      {
        command_if_ios.insert(key);
      }
    }
    for (const auto & key: value.set_after_command_low)
    {
      if(!key.empty())
      {
        command_if_ios.insert(key);
      }
    }
  }

  // get the configurations for different io which needs to be high or low
  for (const auto & [key, val] : params_.configuration_setup.configurations_map)
  {
    config_map_.push_back(val);
  }

  // get the configurations list
  configurations_list_ = params_.configurations;

  for (const auto & config : config_map_)
  {
    for (const auto & io : config.command_high)
    {
      command_if_ios.insert(io);
      state_if_ios.insert(io);
    }
    for (const auto & io : config.command_low)
    {
      command_if_ios.insert(io);
      state_if_ios.insert(io);
    }
    for (const auto & io : config.state_high)
    {
      state_if_ios.insert(io);
    }
    for (const auto & io : config.state_low)
    {
      state_if_ios.insert(io);
    }
  }

  for (size_t i = 0; i < params_.gripper_specific_sensors.size(); ++i)
  {
    state_if_ios.insert(params_.sensors_interfaces.gripper_specific_sensors_map.at(params_.gripper_specific_sensors[i]).input);
  }
}

controller_interface::CallbackReturn IOGripperController::prepare_publishers_and_services()
{

  reconfigureFlag_.store(false);

  // reset service buffer
  service_buffer_.writeFromNonRT(service_mode_type::IDLE);

  // reset gripper state buffer
  gripper_state_buffer_.writeFromNonRT(gripper_state_type::IDLE);

  if (!params_.use_action)
  {
    // callback groups for each service
    open_service_callback_group_ = get_node()->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    close_service_callback_group_ = get_node()->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    reconfigure_service_callback_group_ = get_node()->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // open service
    auto open_service_callback =
      [&](
        const std::shared_ptr<OpenSrvType::Request> /*request*/,
        std::shared_ptr<OpenSrvType::Response> response)
    {
      try
      {
        if (reconfigureFlag_.load())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
          response->success = false;
          return;
        }
        if (closeFlag_.load())
        {
          closeFlag_.store(false);
        }
        openFlag_.store(true);
        service_buffer_.writeFromNonRT(service_mode_type::OPEN);
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
        while(openFlag_.load())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          if ((*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED))
          {
            response->success = false;
            break;
          }
          else {
            response->success = true;
          }
        }
      }
      catch (const std::exception & e)
      {
        response->success = false;
      }
    };

    open_service_ = get_node()->create_service<OpenSrvType>(
      "~/gripper_open", open_service_callback,
      rmw_qos_profile_services_hist_keep_all, open_service_callback_group_);

    // close service
    auto close_service_callback =
      [&](
        const std::shared_ptr<OpenSrvType::Request> /*request*/,
        std::shared_ptr<OpenSrvType::Response> response)
    {
      try
      {
        if (reconfigureFlag_.load())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
          response->success = false;
          return;
        }
        service_buffer_.writeFromNonRT(service_mode_type::CLOSE);
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
        if (openFlag_.load())
        {
          openFlag_.store(false);
        }
        closeFlag_.store(true);
        while(closeFlag_.load())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          if ((*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED))
          {
            response->success = false;
            break;
          }
          else {
            response->success = true;
          }
        }
      }
      catch (const std::exception & e)
      {
        response->success = false;
      }
    };

    close_service_ = get_node()->create_service<OpenSrvType>(
      "~/gripper_close", close_service_callback,
      rmw_qos_profile_services_hist_keep_all, close_service_callback_group_);

    // configure gripper service
    auto configure_gripper_service_callback =
      [&](
        const std::shared_ptr<ConfigSrvType::Request> request,
        std::shared_ptr<ConfigSrvType::Response> response)
    {
      try
      {
        std::string conf = request->config_name;
        configure_gripper_buffer_.writeFromNonRT(conf.c_str());
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
        reconfigureFlag_.store(true);
        while(reconfigureFlag_.load())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        response->result = true;
        response->status = "Gripper reconfigured";
      }
      catch (const std::exception & e)
      {
        response->result = false;
        response->status = "Failed to reconfigure gripper";
      }
    };

    configure_gripper_service_ = get_node()->create_service<ConfigSrvType>(
      "~/reconfigure_to", configure_gripper_service_callback,
      rmw_qos_profile_services_hist_keep_all, reconfigure_service_callback_group_);

  }
  else
  {
    // open close action server
    gripper_feedback_ = std::make_shared<GripperAction::Feedback>();
    gripper_result_ = std::make_shared<GripperAction::Result>();
    gripper_action_server_ = rclcpp_action::create_server<GripperAction>(
      get_node(), "~/gripper_action",
      std::bind(&IOGripperController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&IOGripperController::handle_cancel, this, std::placeholders::_1),
      std::bind(&IOGripperController::handle_accepted, this, std::placeholders::_1));

    // reconfigure action server
    gripper_config_feedback_ = std::make_shared<GripperConfigAction::Feedback>();
    gripper_config_result_ = std::make_shared<GripperConfigAction::Result>();
    gripper_config_action_server_ = rclcpp_action::create_server<GripperConfigAction>(
      get_node(), "~/reconfigure_gripper_action",
      std::bind(&IOGripperController::config_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&IOGripperController::config_handle_cancel, this, std::placeholders::_1),
      std::bind(&IOGripperController::config_handle_accepted, this, std::placeholders::_1));
  }

  try
  {
    // Gripper Joint State publisher
    g_j_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("/joint_states", rclcpp::SystemDefaultsQoS());
    gripper_joint_state_publisher_ = std::make_unique<ControllerStatePublisher>(g_j_s_publisher_);

    auto final_joint_size = params_.open_close_joints.size() + params_.configuration_joints.size();
    
    gripper_joint_state_publisher_->msg_.name.resize(final_joint_size);
    gripper_joint_state_publisher_->msg_.position.resize(final_joint_size);

    joint_state_values_.resize(final_joint_size, std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < params_.open_close_joints.size(); ++i)
    {
      gripper_joint_state_publisher_->msg_.name[i] = params_.open_close_joints[i];
      gripper_joint_state_publisher_->msg_.position[i] = joint_state_values_[i];
    }
    for (size_t i = 0; i < params_.configuration_joints.size(); ++i)
    {
      gripper_joint_state_publisher_->msg_.name[i + params_.open_close_joints.size()] = params_.configuration_joints[i];
      gripper_joint_state_publisher_->msg_.position[i + params_.open_close_joints.size()] = joint_state_values_[i + params_.open_close_joints.size()];
    }
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    // interface publisher
    if_publisher_ = get_node()->create_publisher<InterfaceMsg>(
      "~/dynamic_interface", rclcpp::SystemDefaultsQoS());
    interface_publisher_ = std::make_unique<InterfacePublisher>(if_publisher_);
  }
  catch(const std::exception& e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void IOGripperController::publish_gripper_joint_states()
{
  if (gripper_joint_state_publisher_ && gripper_joint_state_publisher_->trylock())
  {
    gripper_joint_state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();

    // publish gripper joint state values
    for (size_t i = 0; i < joint_state_values_.size(); ++i)
    {
      gripper_joint_state_publisher_->msg_.position[i] = joint_state_values_[i];
    }
   }
  gripper_joint_state_publisher_->unlockAndPublish();

}

void IOGripperController::publish_dynamic_interface_values()
{
  if (interface_publisher_ && interface_publisher_->trylock())
  {
    interface_publisher_->msg_.header.stamp = get_node()->get_clock()->now(); // Make sure it works and discuss with Dr. Denis
    interface_publisher_->msg_.states.interface_names.clear();
    interface_publisher_->msg_.states.values.clear();
    interface_publisher_->msg_.states.values.resize(state_interfaces_.size());
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      interface_publisher_->msg_.states.interface_names.push_back(state_interfaces_.at(i).get_name()); // this can be done in a separate function one time. Change it later TODO (Sachin) :
      interface_publisher_->msg_.states.values.at(i) = static_cast<float>(state_interfaces_.at(i).get_value());
    }

    interface_publisher_->msg_.commands.interface_names.clear();
    interface_publisher_->msg_.commands.values.clear();
    interface_publisher_->msg_.commands.values.resize(command_interfaces_.size());
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      interface_publisher_->msg_.commands.interface_names.push_back(command_interfaces_.at(i).get_name()); // this can be done in a separate function one time. Change it later TODO (Sachin) :
      interface_publisher_->msg_.commands.values.at(i) = static_cast<float>(command_interfaces_.at(i).get_value());
    }
    interface_publisher_->unlockAndPublish();
  }
}


  rclcpp_action::GoalResponse IOGripperController::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal)
    {
      try
      {
        if (reconfigureFlag_.load())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
          return rclcpp_action::GoalResponse::REJECT;
        }
        service_buffer_.writeFromNonRT((goal->open) ? service_mode_type::OPEN : service_mode_type::CLOSE);
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during action handle goal with message: %s", e.what());
        return rclcpp_action::GoalResponse::REJECT;
      }
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

  rclcpp_action::CancelResponse IOGripperController::handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
    {
    (void)goal_handle;
      service_buffer_.writeFromNonRT(service_mode_type::IDLE);
      gripper_state_buffer_.writeFromNonRT(gripper_state_type::IDLE);
      return rclcpp_action::CancelResponse::ACCEPT;
    }

  void IOGripperController::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    std::thread{std::bind(&IOGripperController::execute, this, std::placeholders::_1), goal_handle}.detach();
    
  }

  void IOGripperController::execute(std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    auto result = std::make_shared<GripperAction::Result>();
    auto feedback = std::make_shared<GripperAction::Feedback>();
    while (true)
    {
      if (*(gripper_state_buffer_.readFromRT()) == gripper_state_type::IDLE)
      {
        result->success = true;
        result->message = "Gripper action executed";
        goal_handle->succeed(result);
        break;
      }
      else if (*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED)
      {
        result->success = false;
        result->message = "Gripper action halted";
        goal_handle->abort(result);
        break;
      }
      else 
      {
        feedback->state = static_cast<uint8_t> (*(gripper_state_buffer_.readFromRT()));
        goal_handle->publish_feedback(feedback);
      }
     std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
  }

  rclcpp_action::GoalResponse IOGripperController::config_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperConfigAction::Goal> goal)
    {
      try
    {
      std::string conf = goal->config_name;
      configure_gripper_buffer_.writeFromNonRT(conf.c_str());
      reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
      reconfigureFlag_.store(true);
      
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during action handle goal with message: %s", e.what());
      return rclcpp_action::GoalResponse::REJECT;
      
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


  rclcpp_action::CancelResponse IOGripperController::config_handle_cancel(
    const std::shared_ptr<GoalHandleGripperConfig> goal_handle)
    {
    (void)goal_handle;
    configure_gripper_buffer_.writeFromNonRT("");
    reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
    return rclcpp_action::CancelResponse::ACCEPT;

    }

  void IOGripperController::config_handle_accepted(const std::shared_ptr<GoalHandleGripperConfig> goal_handle)
  {
  std::thread{std::bind(&IOGripperController::config_execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void IOGripperController::config_execute(std::shared_ptr<GoalHandleGripperConfig> goal_handle)
  {
    auto result = std::make_shared<GripperConfigAction::Result>();
    auto feedback = std::make_shared<GripperConfigAction::Feedback>();
    while (true)
    {
      if(*(reconfigure_state_buffer_.readFromRT()) == reconfigure_state_type::IDLE)
      {
        result->result = true;
        result->status = "Gripper reconfigured";
        goal_handle->succeed(result);
        break;
      }
      else 
      {
        feedback->state = static_cast<uint8_t> (*(reconfigure_state_buffer_.readFromRT()));
        goal_handle->publish_feedback(feedback);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }

  void IOGripperController::check_gripper_and_reconfigure_state()
  {
    bool gripper_state_found = false;

    for (size_t i = 0; i < state_ios_open.size(); ++i)
    {
      setResult = find_and_get_state(state_ios_open[i], state_value_);
      if (!setResult)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Failed to get the state for %s", state_ios_open[i].c_str());
      }
      else {
        if (abs(state_value_ - state_ios_open_values[i]) < std::numeric_limits<double>::epsilon())
        {
          gripper_state_found = true;
        }
        else {
          gripper_state_found  = false;
        }
      }
    }

    if (gripper_state_found)
    {
      for (size_t i = 0; i < params_.open.joint_states.size(); ++i)
      {
        joint_state_values_[i] = params_.open.joint_states[i];
      }
    }
    else
    {
      for (const auto & [state_name, state_params] : params_.close.state.possible_closed_states_map)
      {
        for (const auto & high_val : state_params.high)
        {
          setResult = find_and_get_state(high_val, state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", high_val.c_str());
          }
          else {
            if (abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon())
            {
              gripper_state_found = true;
            }
            else {
              gripper_state_found = false;
              break;
            }
          }
        }
        for (const auto & low_val : state_params.low)
        {
          setResult = find_and_get_state(low_val, state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", low_val.c_str());
          }
          else {
            if (abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon())
            {
              gripper_state_found = true;
            }
            else {
              gripper_state_found = false;
              break;
            }
          }
        }

        if (gripper_state_found)
        {
          for (size_t i = 0; i < params_.close.state.possible_closed_states_map.at(state_name).joint_states.size(); ++i)
          {
            joint_state_values_[i] = params_.close.state.possible_closed_states_map.at(state_name).joint_states[i];
          }
          break;
        }
      }
    }

    bool reconfigure_state_found = false;

    for (const auto & [key, val] : params_.configuration_setup.configurations_map)
    {
      for (const auto & io : val.state_high)
      {
        setResult = find_and_get_state(io, state_value_);
        if (!setResult)
        {
          reconfigure_state_found = false;
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else 
        {
            if (!(std::abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon()))
          {
            reconfigure_state_found = false;
            RCLCPP_ERROR(
              get_node()->get_logger(), "value for state doesn't match %s", io.c_str());
            break;
          }
          else {
            reconfigure_state_found = true;
          }
        }
      }

      for (const auto & io : val.state_low)
      {
        setResult = find_and_get_state(io, state_value_);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else
        { 
          if (!(std::abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon()))
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "value doesn't match %s", io.c_str());
            reconfigure_state_found = false;
            break;
          }
          else 
          {
            reconfigure_state_found = true;
          }
        }
      }
      if (reconfigure_state_found)
      {
        for (size_t i = 0; i < val.joint_states.size(); ++i)
        {
            joint_state_values_[i + params_.open_close_joints.size()] = val.joint_states[i];
        }
        break;
      }
    }
  }
}  // namespace io_gripper_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  io_gripper_controller::IOGripperController, controller_interface::ControllerInterface)
