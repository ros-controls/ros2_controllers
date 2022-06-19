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
//
/// \author: Denis Stogl

#ifndef CONTROL_TOOLBOX__PARAMETER_HANDLER_HPP_
#define CONTROL_TOOLBOX__PARAMETER_HANDLER_HPP_

#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rcutils/logging_macros.h"

namespace control_toolbox
{
using rclcpp::ParameterType;

namespace impl
{
inline std::string normalize_params_prefix(std::string prefix)
{
  if (!prefix.empty())
  {
    if ('.' != prefix.back())
    {
      prefix += '.';
    }
  }
  return prefix;
}
}  // namespace impl

struct Parameter
{
  Parameter() = default;

  explicit Parameter(const std::string & name, const uint8_t type, const bool configurable = false)
  : name(name), type(type), configurable(configurable)
  {
  }

  std::string name;
  uint8_t type;
  bool configurable;
};

class ParameterHandler
{
public:
  ParameterHandler(
    const std::string & params_prefix = "", int nr_bool_params = 0, int nr_integer_params = 0,
    int nr_double_params = 0, int nr_string_params = 0, int nr_byte_array_params = 0,
    int nr_bool_array_params = 0, int nr_integer_array_params = 0, int nr_double_array_params = 0,
    int nr_string_array_params = 0);

  virtual ~ParameterHandler() = default;

  void initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  {
    initialize(node->get_node_parameters_interface(), node->get_logger().get_name());
  }

  void initialize(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
    const std::string & logger_name = "::ControllerParameters")
  {
    params_interface_ = node_params;
    logger_name_ = logger_name + "." + "parameters";
  }

  void declare_parameters();

  /**
   * Gets all defined parameters from parameter server after parameters are declared.
   * Optionally, check parameters' validity and update storage.
   *
   * \param[in] check_validity check validity after getting parameters (default: **true**).
   * \param[in] update update parameters in storage after getting them (default: **true**).
   * \return true if all parameters are read successfully, false if a parameter is not provided or
   * parameter configuration is wrong.
   */
  bool get_parameters(bool check_validity = true, bool update = true);

  /**
   * Update all storage variables from the parameter maps.
   * Parameters will be only updated if they are previously read from parameter server or dynamic
   * reconfigure callback occurred.
   *
   * \param[in] check_validity check validity before updating parameters (default: **true**).
   * \return is update was successful, i.e., parameters are valid. If \p check_validity is set
   * **false**, the result is always **true**.
   */
  bool update(bool check_validity = true);

  rcl_interfaces::msg::SetParametersResult set_parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

protected:
  /**
   * Override this method to implement custom parameters check.
   * Default implementation does not checks anything, always returns true.
   *
   * \return **true** if parameters are valid, **false** otherwise.
   */
  virtual bool check_if_parameters_are_valid() { return true; }

  /**
   * Mapping between parameter storage and implementation members.
   * The order of parameter in storage is the order of adding parameters to the storage.
   * E.g., to access the value of 5-th string parameter added to storage use:
   * `fifth_string_param_ = string_parameters_[4].second;
   * where `fifth_string_param_` is the name of the member of a child class.
   */
  virtual void update_storage() = 0;

protected:
  void add_bool_parameter(
    const std::string & name, const bool & configurable = false, const bool & default_value = false)
  {
    bool_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_BOOL, configurable),
       default_value});
  }

  void add_integer_parameter(
    const std::string & name, const bool & configurable = false,
    const int & default_value = std::numeric_limits<int>::quiet_NaN())
  {
    integer_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_INTEGER, configurable),
       default_value});
  }

  void add_double_parameter(
    const std::string & name, const bool & configurable = false,
    const double & default_value = std::numeric_limits<double>::quiet_NaN())
  {
    double_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_DOUBLE, configurable),
       default_value});
  }

  void add_string_parameter(
    const std::string & name, const bool & configurable = false,
    const std::string & default_value = "")
  {
    string_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_STRING, configurable),
       default_value});
  }

  void add_byte_array_parameter(
    const std::string & name, const bool & configurable = false,
    const std::vector<uint8_t> & default_value = {})
  {
    byte_array_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_BYTE_ARRAY, configurable),
       default_value});
  }

  void add_bool_array_parameter(
    const std::string & name, const bool & configurable = false,
    const std::vector<bool> & default_value = {})
  {
    bool_array_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_BOOL_ARRAY, configurable),
       default_value});
  }

  void add_integer_array_parameter(
    const std::string & name, const bool & configurable = false,
    const std::vector<int64_t> & default_value = {})
  {
    integer_array_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_INTEGER_ARRAY, configurable),
       default_value});
  }
  void add_double_array_parameter(
    const std::string & name, const bool & configurable = false,
    const std::vector<double> & default_value = {})
  {
    double_array_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_DOUBLE_ARRAY, configurable),
       default_value});
  }

  void add_string_array_parameter(
    const std::string & name, const bool & configurable = false,
    const std::vector<std::string> & default_value = {})
  {
    string_array_parameters_.push_back(
      {Parameter(params_prefix_ + name, ParameterType::PARAMETER_STRING_ARRAY, configurable),
       default_value});
  }

  template <typename PT>
  bool empty_parameter_in_list(const std::vector<std::pair<Parameter, PT>> & parameters)
  {
    bool ret = false;
    for (const auto & parameter : parameters)
    {
      if (parameter.second.empty())
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "'%s' parameter is empty", parameter.first.name.c_str());
        ret = true;
      }
    }
    return ret;
  }

  bool empty_parameter_in_list(const std::vector<std::pair<Parameter, double>> & parameters)
  {
    return empty_numeric_parameter_in_list(parameters);
  }

  bool empty_parameter_in_list(const std::vector<std::pair<Parameter, int>> & parameters)
  {
    return empty_numeric_parameter_in_list(parameters);
  }

  template <typename PT>
  bool find_and_assign_parameter_value(
    std::vector<std::pair<Parameter, PT>> & parameter_list,
    const rclcpp::Parameter & input_parameter)
  {
    auto is_in_list = [&](const auto & parameter) {
      return parameter.first.name == input_parameter.get_name();
    };

    auto it = std::find_if(parameter_list.begin(), parameter_list.end(), is_in_list);

    if (it != parameter_list.end())
    {
      if (!it->first.configurable)
      {
        throw std::runtime_error(
          "Parameter " + input_parameter.get_name() + " is not configurable.");
      }
      else
      {
        it->second = input_parameter.get_value<PT>();
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "'%s' parameter is updated to value: %s", it->first.name.c_str(),
          input_parameter.value_to_string().c_str());
        return true;
      }
    }
    else
    {
      return false;
    }
  }

  // Storage members
  std::vector<std::pair<Parameter, bool>> bool_parameters_;
  std::vector<std::pair<Parameter, int>> integer_parameters_;
  std::vector<std::pair<Parameter, double>> double_parameters_;
  std::vector<std::pair<Parameter, std::string>> string_parameters_;
  std::vector<std::pair<Parameter, std::vector<uint8_t>>> byte_array_parameters_;
  std::vector<std::pair<Parameter, std::vector<bool>>> bool_array_parameters_;
  std::vector<std::pair<Parameter, std::vector<int64_t>>> integer_array_parameters_;
  std::vector<std::pair<Parameter, std::vector<double>>> double_array_parameters_;
  std::vector<std::pair<Parameter, std::vector<std::string>>> string_array_parameters_;

  // Functional members
  bool declared_;
  bool up_to_date_;
  std::string params_prefix_;

  // Input/Output members to ROS 2
  std::string logger_name_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;

private:
  template <typename PT>
  void declare_parameters_from_list(
    rclcpp::Node::SharedPtr node, const std::vector<std::pair<Parameter, PT>> & parameters)
  {
    for (const auto & parameter : parameters)
    {
      node->declare_parameter<PT>(parameter.first.name, parameter.second);
    }
  }

  template <typename PT>
  void declare_parameters_from_list(const std::vector<std::pair<Parameter, PT>> & parameters)
  {
    for (const auto & parameter : parameters)
    {
      // declare parameter only if it does not already exists
      if (!params_interface_->has_parameter(parameter.first.name))
      {
        rclcpp::ParameterValue default_parameter_value(parameter.second);
        rcl_interfaces::msg::ParameterDescriptor descr;
        descr.name = parameter.first.name;
        descr.type = parameter.first.type;
        descr.read_only = false;

        params_interface_->declare_parameter(parameter.first.name, default_parameter_value, descr);
      }
    }
  }

  template <typename PT>
  bool empty_numeric_parameter_in_list(const std::vector<std::pair<Parameter, PT>> & parameters)
  {
    bool ret = false;
    for (const auto & parameter : parameters)
    {
      if (std::isnan(parameter.second))
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "'%s' parameter is not set", parameter.first.name.c_str());
        ret = true;
      }
    }
    return ret;
  }

  template <typename PT>
  bool get_parameters_from_list(
    const rclcpp::Node::SharedPtr node, std::vector<std::pair<Parameter, PT>> & parameters)
  {
    bool ret = true;
    for (auto & parameter : parameters)
    {
      try
      {
        rclcpp::Parameter input_parameter;  // TODO(destogl): will this be allocated each time?
        ret = node->get_parameter(parameter.first.name, input_parameter);
        parameter.second = input_parameter.get_value<PT>();
      }
      catch (rclcpp::ParameterTypeException & e)
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "'%s' parameter has wrong type", parameter.first.name.c_str());
        ret = false;
        break;
      }
    }
    return ret;
  }

  template <typename PT>
  bool get_parameters_from_list(std::vector<std::pair<Parameter, PT>> & parameters)
  {
    bool ret = true;
    for (auto & parameter : parameters)
    {
      try
      {
        rclcpp::Parameter input_parameter;  // TODO(destogl): will this be allocated each time?
        ret = params_interface_->get_parameter(parameter.first.name, input_parameter);
        parameter.second = input_parameter.get_value<PT>();
      }
      catch (rclcpp::ParameterTypeException & e)
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "'%s' parameter has wrong type", parameter.first.name.c_str());
        ret = false;
        break;
      }
    }
    return ret;
  }
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__PARAMETER_HANDLER_HPP_
