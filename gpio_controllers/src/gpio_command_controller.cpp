// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "gpio_controllers/gpio_command_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"

namespace
{  // utility
template <typename T>
void print_interface(const rclcpp::Logger & logger, const T & command_interfaces)
{
  for (const auto & [interface_name, value] : command_interfaces)
  {
    RCLCPP_ERROR(logger, "Got %s", interface_name.c_str());
  }
}

// called from RT control loop
void reset_controller_reference_msg(
  gpio_controllers::CmdType & msg, const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg.header.stamp = node->now();
  msg.interface_groups.clear();
  msg.interface_values.clear();
}

std::vector<hardware_interface::ComponentInfo> extract_gpios_from_hardware_info(
  const std::vector<hardware_interface::HardwareInfo> & hardware_infos)
{
  std::vector<hardware_interface::ComponentInfo> result;
  for (const auto & hardware_info : hardware_infos)
  {
    std::copy(
      hardware_info.gpios.begin(), hardware_info.gpios.end(), std::back_insert_iterator(result));
  }
  return result;
}
}  // namespace

namespace gpio_controllers
{

GpioCommandController::GpioCommandController() : controller_interface::ControllerInterface() {}

CallbackReturn GpioCommandController::on_init()
try
{
  param_listener_ = std::make_shared<gpio_command_controller_parameters::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}

CallbackReturn GpioCommandController::on_configure(const rclcpp_lifecycle::State &)
try
{
  if (!update_dynamic_map_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  store_command_interface_types();
  store_state_interface_types();

  if (command_interface_types_.empty() && state_interface_types_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command or state interfaces are configured");
    return CallbackReturn::ERROR;
  }

  if (!command_interface_types_.empty())
  {
    gpios_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_.set(*msg); });
  }

  gpio_state_publisher_ =
    get_node()->create_publisher<StateType>("~/gpio_states", rclcpp::SystemDefaultsQoS());

  realtime_gpio_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StateType>>(gpio_state_publisher_);

  action_server_ = rclcpp_action::create_server<GPIOCommandAction>(
    get_node(), "~/gpio_command",
    std::bind(
      &GpioCommandController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GpioCommandController::handle_cancel, this, std::placeholders::_1),
    std::bind(&GpioCommandController::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}

controller_interface::InterfaceConfiguration
GpioCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration GpioCommandController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  return state_interfaces_config;
}

CallbackReturn GpioCommandController::on_activate(const rclcpp_lifecycle::State &)
{
  command_interfaces_map_ =
    create_map_of_references_to_interfaces(command_interface_types_, command_interfaces_);
  state_interfaces_map_ =
    create_map_of_references_to_interfaces(state_interface_types_, state_interfaces_);
  if (
    !check_if_configured_interfaces_matches_received(
      command_interface_types_, command_interfaces_map_) ||
    !check_if_configured_interfaces_matches_received(state_interface_types_, state_interfaces_map_))
  {
    return CallbackReturn::ERROR;
  }

  initialize_gpio_state_msg();
  // Set default value in command
  reset_controller_reference_msg(gpio_commands_, get_node());
  rt_command_.try_set(gpio_commands_);
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GpioCommandController::on_deactivate(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    if (active_goal_handle_ && active_goal_handle_->is_active())
    {
      auto result = std::make_shared<GPIOCommandAction::Result>();
      result->success = false;
      result->message = "Controller deactivated, goal aborted.";
      active_goal_handle_->abort(result);
    }
    active_goal_handle_.reset();
  }
  goal_active_.store(false);
  locked_command_interface_.set(std::string(""));

  // Set default value in command
  reset_controller_reference_msg(gpio_commands_, get_node());
  rt_command_.try_set(gpio_commands_);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GpioCommandController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // Update the monitored state interface value for the active goal.
  if (goal_active_.load())
  {
    auto goal_data_opt = active_goal_data_.try_get();
    if (goal_data_opt.has_value())
    {
      auto & goal_data = goal_data_opt.value();

      // on first update(), send command
      if (!goal_data.command_sent)
      {
        auto cmd_it = command_interfaces_map_.find(goal_data.command_interface);
        if (cmd_it != command_interfaces_map_.end())
        {
          if (!cmd_it->second.get().set_value(goal_data.command_value))
          {
            RCLCPP_WARN(
              get_node()->get_logger(),
              "Action: Unable to set command interface '%s' to value %f.",
              goal_data.command_interface.c_str(), goal_data.command_value);
          }
        }
        goal_data.command_sent = true;
        goal_data.goal_start_time = time.seconds();
        active_goal_data_.try_set(goal_data);
      }

      // Make the state interface value available to the action callback thread.
      auto state_it = state_interfaces_map_.find(goal_data.state_interface);
      if (state_it != state_interfaces_map_.end())
      {
        auto state_val = state_it->second.get().get_optional<double>();
        if (state_val.has_value())
        {
          current_goal_monitored_state_value_.store(state_val.value());
        }
      }
    }
  }

  update_gpios_states();
  return update_gpios_commands();
}

bool GpioCommandController::update_dynamic_map_parameters()
{
  auto logger = get_node()->get_logger();
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();
  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  return true;
}

void GpioCommandController::store_command_interface_types()
{
  // GPIO command interfaces
  for (const auto & [gpio_name, interfaces] : params_.command_interfaces.gpios_map)
  {
    std::transform(
      interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
      std::back_inserter(command_interface_types_),
      [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
  }

  // Joint command interfaces
  for (const auto & [joint_name, interfaces] : params_.command_interfaces.joints_map)
  {
    std::transform(
      interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
      std::back_inserter(command_interface_types_),
      [&](const auto & interface_name) { return joint_name + "/" + interface_name; });
  }
}

bool GpioCommandController::should_broadcast_all_interfaces_of_configured_gpios() const
{
  if (params_.gpios.empty())
  {
    return false;
  }
  auto are_interfaces_empty = [](const auto & interfaces)
  { return interfaces.second.interfaces.empty(); };
  return std::all_of(
    params_.state_interfaces.gpios_map.cbegin(), params_.state_interfaces.gpios_map.cend(),
    are_interfaces_empty);
}

std::vector<hardware_interface::ComponentInfo> GpioCommandController::get_gpios_from_urdf() const
try
{
  return extract_gpios_from_hardware_info(
    hardware_interface::parse_control_resources_from_urdf(get_robot_description()));
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during extracting gpios info from urdf %s \n", e.what());
  return {};
}

void GpioCommandController::set_all_state_interfaces_of_configured_gpios()
{
  const auto gpios{get_gpios_from_urdf()};
  for (const auto & gpio_name : params_.gpios)
  {
    for (const auto & gpio : gpios)
    {
      if (gpio_name == gpio.name)
      {
        std::transform(
          gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
          std::back_insert_iterator(state_interface_types_),
          [&gpio_name](const auto & interface_name)
          { return gpio_name + '/' + interface_name.name; });
      }
    }
  }
}

void GpioCommandController::store_state_interface_types()
{
  if (should_broadcast_all_interfaces_of_configured_gpios())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "GPIO state interfaces are not configured. All available interfaces of configured GPIOs will "
      "be broadcasted.");
    set_all_state_interfaces_of_configured_gpios();
  }
  else
  {
    for (const auto & [gpio_name, interfaces] : params_.state_interfaces.gpios_map)
    {
      std::transform(
        interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
        std::back_inserter(state_interface_types_),
        [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
    }
  }

  // Joint state interfaces
  for (const auto & [joint_name, interfaces] : params_.state_interfaces.joints_map)
  {
    std::transform(
      interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
      std::back_inserter(state_interface_types_),
      [&](const auto & interface_name) { return joint_name + "/" + interface_name; });
  }
}

// TODO: rename these GPIO-specific functions; they now handle joints too.
void GpioCommandController::initialize_gpio_state_msg()
{
  // Combine GPIOs and joints into one list for state publishing.
  std::vector<std::string> all_components;
  all_components.insert(all_components.end(), params_.gpios.begin(), params_.gpios.end());
  all_components.insert(all_components.end(), params_.joints.begin(), params_.joints.end());

  gpio_state_msg_.header.stamp = get_node()->now();
  gpio_state_msg_.interface_groups.resize(all_components.size());
  gpio_state_msg_.interface_values.resize(all_components.size());

  for (std::size_t index = 0; index < all_components.size(); ++index)
  {
    const auto & component_name = all_components[index];
    gpio_state_msg_.interface_groups[index] = component_name;
    gpio_state_msg_.interface_values[index].interface_names =
      get_gpios_state_interfaces_names(component_name);
    gpio_state_msg_.interface_values[index].values = std::vector<double>(
      gpio_state_msg_.interface_values[index].interface_names.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
}

InterfacesNames GpioCommandController::get_gpios_state_interfaces_names(
  const std::string & gpio_name) const
{
  InterfacesNames result;
  for (const auto & interface_name : state_interface_types_)
  {
    const auto it = state_interfaces_map_.find(interface_name);
    if (it != state_interfaces_map_.cend() && it->second.get().get_prefix_name() == gpio_name)
    {
      result.emplace_back(it->second.get().get_interface_name());
    }
  }
  return result;
}

template <typename T>
std::unordered_map<std::string, std::reference_wrapper<T>>
GpioCommandController::create_map_of_references_to_interfaces(
  const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces)
{
  std::unordered_map<std::string, std::reference_wrapper<T>> map;
  for (const auto & interface_name : interfaces_from_params)
  {
    auto interface = std::find_if(
      configured_interfaces.begin(), configured_interfaces.end(),
      [&](const auto & configured_interface)
      {
        const auto full_name_interface_name = configured_interface.get_name();
        return full_name_interface_name == interface_name;
      });
    if (interface != configured_interfaces.end())
    {
      map.emplace(interface_name, std::ref(*interface));
    }
  }
  return map;
}

template <typename T>
bool GpioCommandController::check_if_configured_interfaces_matches_received(
  const InterfacesNames & interfaces_from_params, const T & configured_interfaces)
{
  if (!(configured_interfaces.size() == interfaces_from_params.size()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu interfaces, got %zu", interfaces_from_params.size(),
      configured_interfaces.size());
    for (const auto & interface : interfaces_from_params)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %s", interface.c_str());
    }
    print_interface(get_node()->get_logger(), configured_interfaces);
    return false;
  }
  return true;
}

controller_interface::return_type GpioCommandController::update_gpios_commands()
{
  auto gpio_commands_op = rt_command_.try_get();
  if (gpio_commands_op.has_value())
  {
    gpio_commands_ = gpio_commands_op.value();
  }
  if (gpio_commands_.interface_groups.empty() || gpio_commands_.interface_values.empty())
  {
    // no command received yet
    return controller_interface::return_type::OK;
  }

  // Command interface locked by the action, if any.
  std::string locked_interface;
  if (goal_active_.load())
  {
    auto locked_opt = locked_command_interface_.try_get();
    if (locked_opt.has_value())
    {
      locked_interface = locked_opt.value();
    }
  }

  for (std::size_t gpio_index = 0; gpio_index < gpio_commands_.interface_groups.size();
       ++gpio_index)
  {
    const auto & gpio_name = gpio_commands_.interface_groups[gpio_index];
    if (
      gpio_commands_.interface_values[gpio_index].values.size() !=
      gpio_commands_.interface_values[gpio_index].interface_names.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "For gpio %s interfaces_names do not match values",
        gpio_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    for (std::size_t command_interface_index = 0;
         command_interface_index < gpio_commands_.interface_values[gpio_index].values.size();
         ++command_interface_index)
    {
      const auto full_command_interface_name =
        gpio_name + '/' +
        gpio_commands_.interface_values[gpio_index].interface_names[command_interface_index];

      if (!locked_interface.empty() && full_command_interface_name == locked_interface)
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(),
          "Skipping topic command for '%s' - interface is being used by an active action goal.",
          full_command_interface_name.c_str());
        continue;
      }

      apply_command(gpio_commands_, gpio_index, command_interface_index);
    }
  }
  return controller_interface::return_type::OK;
}

void GpioCommandController::apply_command(
  const CmdType & gpio_commands, std::size_t gpio_index, std::size_t command_interface_index) const
{
  const auto full_command_interface_name =
    gpio_commands.interface_groups[gpio_index] + '/' +
    gpio_commands.interface_values[gpio_index].interface_names[command_interface_index];

  const auto & command_value =
    gpio_commands.interface_values[gpio_index].values[command_interface_index];

  try
  {
    if (!command_interfaces_map_.at(full_command_interface_name).get().set_value(command_value))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Unable to set the command for interface '%s' with value '%f'.",
        full_command_interface_name.c_str(), command_value);
    }
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during applying command stage of %s with message: %s \n",
      full_command_interface_name.c_str(), e.what());
  }
}

void GpioCommandController::update_gpios_states()
{
  if (!realtime_gpio_state_publisher_)
  {
    return;
  }

  gpio_state_msg_.header.stamp = get_node()->now();
  for (std::size_t gpio_index = 0; gpio_index < gpio_state_msg_.interface_groups.size();
       ++gpio_index)
  {
    for (std::size_t interface_index = 0;
         interface_index < gpio_state_msg_.interface_values[gpio_index].interface_names.size();
         ++interface_index)
    {
      apply_state_value(gpio_state_msg_, gpio_index, interface_index);
    }
  }
  realtime_gpio_state_publisher_->try_publish(gpio_state_msg_);
}

void GpioCommandController::apply_state_value(
  StateType & state_msg, std::size_t gpio_index, std::size_t interface_index) const
{
  const auto interface_name =
    state_msg.interface_groups[gpio_index] + '/' +
    state_msg.interface_values[gpio_index].interface_names[interface_index];
  try
  {
    auto state_msg_interface_value_op =
      state_interfaces_map_.at(interface_name).get().get_optional();

    if (!state_msg_interface_value_op.has_value())
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Unable to retrieve the data from state: %s \n",
        interface_name.c_str());
    }
    else
    {
      state_msg.interface_values[gpio_index].values[interface_index] =
        state_msg_interface_value_op.value();
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during reading state of: %s \n", interface_name.c_str());
  }
}

// Action server
// ==========================
rclcpp_action::GoalResponse GpioCommandController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GPIOCommandAction::Goal> goal)
{
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Received GPIO command action goal: command_interface='%s', command_value=%f, "
    "state_interface='%s', state_value=%f, tolerance=%f, timeout=%f",
    goal->command_interface.c_str(), goal->command_value,
    goal->state_interface.c_str(), goal->state_value,
    goal->tolerance, goal->timeout);

  if (command_interfaces_map_.find(goal->command_interface) == command_interfaces_map_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Action goal rejected: command_interface '%s' not found in claimed interfaces.",
      goal->command_interface.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (state_interfaces_map_.find(goal->state_interface) == state_interfaces_map_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Action goal rejected: state_interface '%s' not found in claimed interfaces.",
      goal->state_interface.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal_active_.load())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Preempting currently active goal to accept new goal on '%s'.",
      goal->command_interface.c_str());
    preempt_requested_.store(true);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GpioCommandController::handle_cancel(
  std::shared_ptr<GoalHandleGPIOCommand> /*goal_handle*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel GPIO command action goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GpioCommandController::handle_accepted(std::shared_ptr<GoalHandleGPIOCommand> goal_handle)
{
  while (goal_active_.load() && preempt_requested_.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    if (active_goal_handle_ && active_goal_handle_->is_active())
    {
      auto result = std::make_shared<GPIOCommandAction::Result>();
      result->success = false;
      result->message = "Goal preempted by a new goal.";
      active_goal_handle_->abort(result);
      RCLCPP_INFO(get_node()->get_logger(), "Previous goal preempted.");
    }
    active_goal_handle_ = goal_handle;
  }

  const auto goal = goal_handle->get_goal();

  ActiveGoalData goal_data;
  goal_data.command_interface = goal->command_interface;
  goal_data.command_value = goal->command_value;
  goal_data.state_interface = goal->state_interface;
  goal_data.state_value = goal->state_value;
  goal_data.tolerance = goal->tolerance;
  goal_data.timeout = goal->timeout;
  goal_data.command_sent = false;

  locked_command_interface_.set(goal->command_interface);
  active_goal_data_.set(goal_data);
  current_goal_monitored_state_value_.store(std::numeric_limits<double>::quiet_NaN());
  preempt_requested_.store(false);
  goal_active_.store(true);

  double timeout = goal->timeout;
  if (timeout <= 0.0)
  {
    timeout = params_.action_timeout;
  }

  auto feedback = std::make_shared<GPIOCommandAction::Feedback>();
  auto result = std::make_shared<GPIOCommandAction::Result>();

  const auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Goal canceled.";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_node()->get_logger(), "GPIO command action goal canceled.");
      break;
    }

    if (preempt_requested_.load())
    {
      // handle_accepted() of the new goal aborts this one.
      break;
    }

    const double current_val = current_goal_monitored_state_value_.load();

    if (!std::isnan(current_val))
    {
      feedback->current_state = current_val;
      goal_handle->publish_feedback(feedback);

      if (std::abs(current_val - goal->state_value) <= goal->tolerance)
      {
        result->success = true;
        result->message = "State interface '" + goal->state_interface +
                          "' reached target value " + std::to_string(goal->state_value) +
                          " (current: " + std::to_string(current_val) + ").";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_node()->get_logger(), "%s", result->message.c_str());
        break;
      }
    }

    if (timeout > 0.0)
    {
      const auto elapsed = std::chrono::steady_clock::now() - start_time;
      const double elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
      if (elapsed_seconds > timeout)
      {
        result->success = false;
        result->message = "Goal timed out after " + std::to_string(timeout) +
                          " seconds. State interface '" + goal->state_interface +
                          "' did not converge (current: " +
                          std::to_string(current_val) + ", target: " +
                          std::to_string(goal->state_value) + ").";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_node()->get_logger(), "%s", result->message.c_str());
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  goal_active_.store(false);
  locked_command_interface_.set(std::string(""));
  preempt_requested_.store(false);

  std::lock_guard<std::mutex> lock(goal_handle_mutex_);
  if (active_goal_handle_ == goal_handle)
  {
    active_goal_handle_.reset();
  }
}

}  // namespace gpio_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gpio_controllers::GpioCommandController, controller_interface::ControllerInterface)
