// Copyright 2025 ros2_control PMC
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

#include "chained_filter_controller/chained_filter.hpp"

#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/version.h"

namespace chained_filter_controller
{

controller_interface::CallbackReturn ChainedFilter::on_init()
{
  try
  {
    param_listener_ = std::make_shared<chained_filter::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChainedFilter::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, params_.input_interfaces};
}

controller_interface::CallbackReturn ChainedFilter::on_configure(const rclcpp_lifecycle::State &)
{
  try
  {
    params_ = param_listener_->get_params();

    if (params_.input_interfaces.size() != params_.output_interfaces.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Input and output interfaces have to have the same size.");
      return controller_interface::CallbackReturn::FAILURE;
    }
    filters_.resize(params_.input_interfaces.size());
    output_state_values_.resize(
      params_.output_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

    // Check if global filter chain configuration is set, if not we use a per-interface
    // configuration
    bool has_global_filter_setting = true;
    if (params_.input_interfaces.size() > 1)
    {
      if (!get_node()->has_parameter("filter_chain.filter1.name"))
      {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = "filter_chain.filter1.name";
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        desc.read_only = true;
        get_node()->declare_parameter(
          "filter_chain.filter1.name", rclcpp::ParameterValue{""}, desc);
        has_global_filter_setting =
          !get_node()->get_parameter("filter_chain.filter1.name").as_string().empty();
      }
      RCLCPP_INFO_EXPRESSION(
        get_node()->get_logger(), has_global_filter_setting,
        "Using global filter setting for all interfaces.");
    }

    for (size_t i = 0; i < params_.input_interfaces.size(); ++i)
    {
      filters_[i] = std::make_unique<filters::FilterChain<double>>("double");
      if (!filters_[i]->configure(
            has_global_filter_setting ? "filter_chain"
                                      : params_.input_interfaces[i] + ".filter_chain",
            get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to configure filter chain for interfaces  %s. Check the parameters for filters "
          "setup.",
          params_.input_interfaces[i].c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(
    output_state_values_.begin(), output_state_values_.end(),
    std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < params_.input_interfaces.size(); ++i)
  {
    const auto sensor_op = state_interfaces_[i].get_optional();
    if (!sensor_op.has_value())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to read sensor value from state interface '%s'.",
        state_interfaces_[0].get_name().c_str());
      continue;
    }
    if (std::isfinite(sensor_op.value()))
    {
      filters_[i]->update(sensor_op.value(), output_state_values_[i]);
    }
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(params_.output_interfaces.size());
  for (size_t i = 0; i < params_.output_interfaces.size(); ++i)
  {
    // Split the output interface name by the last "/" if present
    const std::string & full_name = params_.output_interfaces.at(i);
    auto pos = full_name.rfind('/');
    const std::string iface_name =
      (pos != std::string::npos) ? full_name.substr(pos + 1) : full_name;
    const std::string iface_prefix =
      (pos != std::string::npos) ? "/" + full_name.substr(0, pos) : "";

    state_interfaces.emplace_back(
      get_node()->get_name() + iface_prefix, iface_name, &output_state_values_.at(i));
  }
  return state_interfaces;
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

}  // namespace chained_filter_controller

PLUGINLIB_EXPORT_CLASS(
  chained_filter_controller::ChainedFilter, controller_interface::ChainableControllerInterface)
