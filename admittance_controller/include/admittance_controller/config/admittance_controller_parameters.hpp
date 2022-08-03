// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

namespace gen_param_struct_validators {
  class Result {
  public:
    template <typename... Args>
    Result(const std::string& format, Args... args) {
      msg_ = fmt::format(format, args...);
      success_ = false;
    }

    Result() = default;

    operator rcl_interfaces::msg::SetParametersResult() const {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = success_;
      result.reason = msg_;
      return result;
    }

    bool success() { return success_; }

    std::string error_msg() { return msg_; }

  private:
    std::string msg_;
    bool success_ = true;
  };

  auto OK = Result();
  using ERROR = Result;

  template <typename T>
  bool contains(std::vector<T> const& vec, T const& val) {
    return std::find(vec.cbegin(), vec.cend(), val) != vec.cend();
  }

  template <class T>
  bool is_unique(std::vector<T> const& x) {
    auto vec = x;
    std::sort(vec.begin(), vec.end());
    return std::adjacent_find(vec.cbegin(), vec.cend()) == vec.cend();
  }

  template <typename T>
  Result unique(rclcpp::Parameter const& parameter) {
    if (!is_unique<T>(parameter.get_value<std::vector<T>>())) {
      return ERROR("Parameter '{}' must only contain unique values",
                   parameter.get_name());
    }
    return OK;
  }

  template <typename T>
  Result subset_of(rclcpp::Parameter const& parameter,
                   std::vector<T> valid_values) {
    auto const& input_values = parameter.get_value<std::vector<T>>();

    for (auto const& value : input_values) {
      if (!contains(valid_values, value)) {
        return ERROR("Invalid entry '{}' for parameter '{}'. Not in set: {}",
                     value, parameter.get_name(), valid_values);
      }
    }

    return OK;
  }

  template <typename T>
  Result fixed_size(const rclcpp::Parameter& parameter, size_t size) {
    auto param_value = parameter.get_value<std::vector<T>>();
    if (param_value.size() != size) {
      return ERROR("Invalid length '{}' for parameter '{}'. Required length: {}",
                   param_value.size(), parameter.get_name().c_str(), size);
    }
    return OK;
  }

  template <typename T>
  Result size_gt(rclcpp::Parameter const& parameter, size_t size) {
    auto const& values = parameter.get_value<std::vector<T>>();
    if (values.size() > size) {
      return OK;
    }

    return ERROR(
        "Invalid length '{}' for parameter '{}'. Required greater than: {}",
        values.size(), parameter.get_name(), size);
  }

  template <typename T>
  Result size_lt(rclcpp::Parameter const& parameter, size_t size) {
    auto const& values = parameter.get_value<std::vector<T>>();
    if (values.size() < size) {
      return OK;
    }

    return ERROR("Invalid length '{}' for parameter '{}'. Required less than: {}",
                 values.size(), parameter.get_name(), size);
  }

  template <typename T>
  Result element_bounds(const rclcpp::Parameter& parameter, T lower, T upper) {
    auto param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value) {
      if (val < lower || val > upper) {
        return ERROR(
            "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
            val, parameter.get_name(), lower, upper);
      }
    }
    return OK;
  }

  template <typename T>
  Result lower_element_bounds(const rclcpp::Parameter& parameter, T lower) {
    auto param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value) {
      if (val < lower) {
        return ERROR(
            "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
            val, parameter.get_name(), lower);
      }
    }
    return OK;
  }

  template <typename T>
  Result upper_element_bounds(const rclcpp::Parameter& parameter, T upper) {
    auto param_value = parameter.get_value<std::vector<T>>();
    for (auto val : param_value) {
      if (val > upper) {
        return ERROR(
            "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
            val, parameter.get_name(), upper);
      }
    }
    return OK;
  }

  template <typename T>
  Result bounds(const rclcpp::Parameter& parameter, T lower, T upper) {
    auto param_value = parameter.get_value<T>();
    if (param_value < lower || param_value > upper) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
          param_value, parameter.get_name(), lower, upper);
    }
    return OK;
  }

  template <typename T>
  Result lower_bounds(const rclcpp::Parameter& parameter, T lower) {
    auto param_value = parameter.get_value<T>();
    if (param_value < lower) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
          param_value, parameter.get_name(), lower);
    }
    return OK;
  }

  template <typename T>
  Result upper_bounds(const rclcpp::Parameter& parameter, T upper) {
    auto param_value = parameter.get_value<T>();
    if (param_value > upper) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
          param_value, parameter.get_name(), upper);
    }
    return OK;
  }

  template <typename T>
  Result one_of(rclcpp::Parameter const& parameter, std::vector<T> collection) {
    auto param_value = parameter.get_value<T>();

    if (std::find(collection.cbegin(), collection.cend(), param_value) ==
        collection.end()) {
      return ERROR("The parameter '{}' with the value '{}' not in the set: {}",
                   parameter.get_name(), param_value,
                   fmt::format("{}", fmt::join(collection, ", ")));
    }

    return OK;
  }

} // namespace gen_param_struct_validators

namespace admittance_controller {
  struct Params {
    std::vector<std::string> joints;
    std::vector<std::string> command_interfaces;
    std::vector<std::string> state_interfaces;
    std::vector<std::string> chainable_command_interfaces;
    std::string robot_description;
    bool enable_parameter_update_without_reactivation = true;
    struct Kinematics {
      std::string plugin_name;
      std::string plugin_package;
      std::string base;
      std::string tip;
      double alpha = 0.0005;
    } kinematics;
    struct FtSensor {
      std::string name;
      double filter_coefficient = 0.005;
      struct Frame {
        std::string id;
        bool external = false;
      } frame;
    } ft_sensor;
    struct Control {
      struct Frame {
        std::string id;
        bool external = false;
      } frame;
    } control;
    struct FixedWorldFrame {
      struct Frame {
        std::string id;
        bool external = false;
      } frame;
    } fixed_world_frame;
    struct GravityCompensation {
      struct Frame {
        std::string id;
        bool external = false;
      } frame;
      struct Cog {
        std::vector<double> pos;
        double force = 0.0;
      } CoG;
    } gravity_compensation;
    struct Admittance {
      std::vector<bool> selected_axes;
      std::vector<double> mass;
      std::vector<double> damping_ratio;
      std::vector<double> stiffness;
    } admittance;
    // for detecting if the parameter struct has been updated
    rclcpp::Time __stamp;
  };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node)
        : ParamListener(node->get_node_parameters_interface()) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ParamListener(node->get_node_parameters_interface()) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface){
      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      return params_;
    }

    bool is_old(Params const& other) const {
      return params_.__stamp != other.__stamp;
    }

    void refresh_dynamic_parameters() {
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      // Copy params_ so we only update it if all validation succeeds
      Params updated_params = params_;

      for (const auto &param: parameters) {
        if (param.get_name() == "joints") {
          updated_params.joints = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces") {
          updated_params.command_interfaces = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces") {
          updated_params.state_interfaces = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces") {
          updated_params.chainable_command_interfaces = param.as_string_array();
        }
        if (param.get_name() == "kinematics.plugin_name") {
          updated_params.kinematics.plugin_name = param.as_string();
        }
        if (param.get_name() == "kinematics.plugin_package") {
          updated_params.kinematics.plugin_package = param.as_string();
        }
        if (param.get_name() == "kinematics.base") {
          updated_params.kinematics.base = param.as_string();
        }
        if (param.get_name() == "kinematics.tip") {
          updated_params.kinematics.tip = param.as_string();
        }
        if (param.get_name() == "kinematics.alpha") {
          updated_params.kinematics.alpha = param.as_double();
        }
        if (param.get_name() == "ft_sensor.name") {
          updated_params.ft_sensor.name = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.id") {
          updated_params.ft_sensor.frame.id = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.external") {
          updated_params.ft_sensor.frame.external = param.as_bool();
        }
        if (param.get_name() == "ft_sensor.filter_coefficient") {
          updated_params.ft_sensor.filter_coefficient = param.as_double();
        }
        if (param.get_name() == "control.frame.id") {
          updated_params.control.frame.id = param.as_string();
        }
        if (param.get_name() == "control.frame.external") {
          updated_params.control.frame.external = param.as_bool();
        }
        if (param.get_name() == "fixed_world_frame.frame.id") {
          updated_params.fixed_world_frame.frame.id = param.as_string();
        }
        if (param.get_name() == "fixed_world_frame.frame.external") {
          updated_params.fixed_world_frame.frame.external = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.frame.id") {
          updated_params.gravity_compensation.frame.id = param.as_string();
        }
        if (param.get_name() == "gravity_compensation.frame.external") {
          updated_params.gravity_compensation.frame.external = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.CoG.pos") {
          if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 3);
              !validation_result.success()) {
            return validation_result;
          }
          updated_params.gravity_compensation.CoG.pos = param.as_double_array();
        }
        if (param.get_name() == "gravity_compensation.CoG.force") {
          updated_params.gravity_compensation.CoG.force = param.as_double();
        }
        if (param.get_name() == "admittance.selected_axes") {
          if(auto validation_result = gen_param_struct_validators::fixed_size<bool>(param, 6);
              !validation_result.success()) {
            return validation_result;
          }
          updated_params.admittance.selected_axes = param.as_bool_array();
        }
        if (param.get_name() == "admittance.mass") {
          if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
              !validation_result.success()) {
            return validation_result;
          }
          if(auto validation_result = gen_param_struct_validators::element_bounds<double>(param, 0.0001, 1000000.0);
              !validation_result.success()) {
            return validation_result;
          }
          updated_params.admittance.mass = param.as_double_array();
        }
        if (param.get_name() == "admittance.damping_ratio") {
          if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
              !validation_result.success()) {
            return validation_result;
          }
          updated_params.admittance.damping_ratio = param.as_double_array();
        }
        if (param.get_name() == "admittance.stiffness") {
          if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
              !validation_result.success()) {
            return validation_result;
          }
          if(auto validation_result = gen_param_struct_validators::element_bounds<double>(param, 0.0, 100000000.0);
              !validation_result.success()) {
            return validation_result;
          }
          updated_params.admittance.stiffness = param.as_double_array();
        }
        if (param.get_name() == "robot_description") {
          updated_params.robot_description = param.as_string();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          updated_params.enable_parameter_update_without_reactivation = param.as_bool();
        }
      }

      updated_params.__stamp = clock_.now();
      params_ = updated_params;
      return gen_param_struct_validators::OK;
    }

    void declare_params(){
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter("joints")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which joints will be used by the controller";
        descriptor.read_only = true;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
        parameters_interface_->declare_parameter("joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("command_interfaces")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which command interfaces to claim";
        descriptor.read_only = true;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
        parameters_interface_->declare_parameter("command_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("state_interfaces")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which state interfaces to claim";
        descriptor.read_only = true;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
        parameters_interface_->declare_parameter("state_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("chainable_command_interfaces")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which chainable interfaces to claim";
        descriptor.read_only = true;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
        parameters_interface_->declare_parameter("chainable_command_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("kinematics.plugin_name")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which kinematics plugin to load";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("kinematics.plugin_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("kinematics.plugin_package")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the package to load the kinematics plugin from";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("kinematics.plugin_package", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("kinematics.base")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the base link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("kinematics.base", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("kinematics.tip")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the end effector link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("kinematics.tip", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("kinematics.alpha")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the damping coefficient for the Jacobian pseudo inverse";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.kinematics.alpha);
        parameters_interface_->declare_parameter("kinematics.alpha", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("ft_sensor.name")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "name of the force torque sensor in the robot description";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("ft_sensor.name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("ft_sensor.frame.id")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame of the force torque sensor";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("ft_sensor.frame.id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("ft_sensor.frame.external")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the force torque sensor is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.ft_sensor.frame.external);
        parameters_interface_->declare_parameter("ft_sensor.frame.external", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("ft_sensor.filter_coefficient")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the coefficient for the sensor's exponential filter";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.ft_sensor.filter_coefficient);
        parameters_interface_->declare_parameter("ft_sensor.filter_coefficient", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("control.frame.id")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "control frame used for admittance control";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("control.frame.id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("control.frame.external")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the control frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.control.frame.external);
        parameters_interface_->declare_parameter("control.frame.external", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("fixed_world_frame.frame.id")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "world frame, gravity points down (neg. Z) in this frame";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("fixed_world_frame.frame.id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("fixed_world_frame.frame.external")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the world frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.fixed_world_frame.frame.external);
        parameters_interface_->declare_parameter("fixed_world_frame.frame.external", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("gravity_compensation.frame.id")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame which center of gravity (CoG) is defined in";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("gravity_compensation.frame.id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("gravity_compensation.frame.external")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the center of gravity frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.gravity_compensation.frame.external);
        parameters_interface_->declare_parameter("gravity_compensation.frame.external", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("gravity_compensation.CoG.pos")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "position of the center of gravity (CoG) in its frame";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        parameters_interface_->declare_parameter("gravity_compensation.CoG.pos", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("gravity_compensation.CoG.force")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "weight of the end effector, e.g mass * 9.81";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.gravity_compensation.CoG.force);
        parameters_interface_->declare_parameter("gravity_compensation.CoG.force", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("admittance.selected_axes")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the axes x, y, z, rx, ry, and rz are enabled";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
        parameters_interface_->declare_parameter("admittance.selected_axes", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("admittance.mass")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies mass values for x, y, z, rx, ry, and rz used in the admittance calculation";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        parameters_interface_->declare_parameter("admittance.mass", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("admittance.damping_ratio")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        parameters_interface_->declare_parameter("admittance.damping_ratio", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("admittance.stiffness")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies stiffness values for x, y, z, rx, ry, and rz used in the admittance calculation";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        parameters_interface_->declare_parameter("admittance.stiffness", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("robot_description")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "Contains robot description in URDF format. The description is used to perform forward and inverse kinematics.";
        descriptor.read_only = true;
        auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
        parameters_interface_->declare_parameter("robot_description", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter("enable_parameter_update_without_reactivation")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "if enabled, parameters will be dynamically updated in the control loop";
        descriptor.read_only = false;
        auto parameter = rclcpp::ParameterValue(params_.enable_parameter_update_without_reactivation);
        parameters_interface_->declare_parameter("enable_parameter_update_without_reactivation", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter("joints");
      params_.joints = param.as_string_array();
      param = parameters_interface_->get_parameter("command_interfaces");
      params_.command_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter("state_interfaces");
      params_.state_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter("chainable_command_interfaces");
      params_.chainable_command_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter("kinematics.plugin_name");
      params_.kinematics.plugin_name = param.as_string();
      param = parameters_interface_->get_parameter("kinematics.plugin_package");
      params_.kinematics.plugin_package = param.as_string();
      param = parameters_interface_->get_parameter("kinematics.base");
      params_.kinematics.base = param.as_string();
      param = parameters_interface_->get_parameter("kinematics.tip");
      params_.kinematics.tip = param.as_string();
      param = parameters_interface_->get_parameter("kinematics.alpha");
      params_.kinematics.alpha = param.as_double();
      param = parameters_interface_->get_parameter("ft_sensor.name");
      params_.ft_sensor.name = param.as_string();
      param = parameters_interface_->get_parameter("ft_sensor.frame.id");
      params_.ft_sensor.frame.id = param.as_string();
      param = parameters_interface_->get_parameter("ft_sensor.frame.external");
      params_.ft_sensor.frame.external = param.as_bool();
      param = parameters_interface_->get_parameter("ft_sensor.filter_coefficient");
      params_.ft_sensor.filter_coefficient = param.as_double();
      param = parameters_interface_->get_parameter("control.frame.id");
      params_.control.frame.id = param.as_string();
      param = parameters_interface_->get_parameter("control.frame.external");
      params_.control.frame.external = param.as_bool();
      param = parameters_interface_->get_parameter("fixed_world_frame.frame.id");
      params_.fixed_world_frame.frame.id = param.as_string();
      param = parameters_interface_->get_parameter("fixed_world_frame.frame.external");
      params_.fixed_world_frame.frame.external = param.as_bool();
      param = parameters_interface_->get_parameter("gravity_compensation.frame.id");
      params_.gravity_compensation.frame.id = param.as_string();
      param = parameters_interface_->get_parameter("gravity_compensation.frame.external");
      params_.gravity_compensation.frame.external = param.as_bool();
      param = parameters_interface_->get_parameter("gravity_compensation.CoG.pos");
      if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 3);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'gravity_compensation.CoG.pos': " + validation_result.error_msg()));
      }
      params_.gravity_compensation.CoG.pos = param.as_double_array();
      param = parameters_interface_->get_parameter("gravity_compensation.CoG.force");
      params_.gravity_compensation.CoG.force = param.as_double();
      param = parameters_interface_->get_parameter("admittance.selected_axes");
      if(auto validation_result = gen_param_struct_validators::fixed_size<bool>(param, 6);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.selected_axes': " + validation_result.error_msg()));
      }
      params_.admittance.selected_axes = param.as_bool_array();
      param = parameters_interface_->get_parameter("admittance.mass");
      if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.mass': " + validation_result.error_msg()));
      }
      if(auto validation_result = gen_param_struct_validators::element_bounds<double>(param, 0.0001, 1000000.0);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.mass': " + validation_result.error_msg()));
      }
      params_.admittance.mass = param.as_double_array();
      param = parameters_interface_->get_parameter("admittance.damping_ratio");
      if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.damping_ratio': " + validation_result.error_msg()));
      }
      params_.admittance.damping_ratio = param.as_double_array();
      param = parameters_interface_->get_parameter("admittance.stiffness");
      if(auto validation_result = gen_param_struct_validators::fixed_size<double>(param, 6);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.stiffness': " + validation_result.error_msg()));
      }
      if(auto validation_result = gen_param_struct_validators::element_bounds<double>(param, 0.0, 100000000.0);
          !validation_result.success()) {
        throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'admittance.stiffness': " + validation_result.error_msg()));
      }
      params_.admittance.stiffness = param.as_double_array();
      param = parameters_interface_->get_parameter("robot_description");
      params_.robot_description = param.as_string();
      param = parameters_interface_->get_parameter("enable_parameter_update_without_reactivation");
      params_.enable_parameter_update_without_reactivation = param.as_bool();


      params_.__stamp = clock_.now();
    }

  private:
    Params params_;
    rclcpp::Clock clock_;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
  };

} // namespace admittance_controller