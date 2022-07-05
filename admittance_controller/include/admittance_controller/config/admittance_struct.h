// this is auto-generated code 

#include <rclcpp/node.hpp>
#include <vector>
#include <string>


namespace admittance_struct_parameters {

  struct admittance_struct {
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;

    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    admittance_struct(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      declare_params(parameters_interface);
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters) {
        return this->update(parameters);
      };
      handle_ = parameters_interface->add_on_set_parameters_callback(update_param_cb);
    }

    void
    declare_overrides(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface,
                      const std::map<std::string, rclcpp::ParameterValue> &overrides,
                      std::unordered_map<std::string, rcl_interfaces::msg::ParameterDescriptor> &desc_map) {

      for (auto &key: overrides) {
        if (!parameters_interface->has_parameter(key.first)) {
          parameters_interface->declare_parameter(key.first, key.second, desc_map[key.first]);
        }
      }
    }

    template<typename T>
    bool validate_length(const std::vector<T> &values, size_t len) {
      return values.size() == len;
    }

    template<typename T>
    bool validate_length(const std::string &name, const std::vector<T> &values, size_t len,
                         rcl_interfaces::msg::SetParametersResult &result) {
      if (!validate_length(values, len)) {
        result.reason = std::string("Invalid size for vector parameter ") + name +
                        ". Expected " + std::to_string(len) + " got " + std::to_string(values.size());
        return false;
      }
      return true;
    }


    template<typename T>
    bool validate_bounds(std::vector<T> values, const T &lower_bound, const T &upper_bound) {
      for (const auto &val: values) {
        if (!validate_bounds(val, lower_bound, upper_bound)) {
          return false;
        }
      }
      return true;
    }

    template<typename T>
    bool validate_bounds(const std::string &name, std::vector<T> values, const T &lower_bound, const T &upper_bound,
                         rcl_interfaces::msg::SetParametersResult &result) {
      for (const auto &val: values) {
        if (!validate_bounds(name, val, lower_bound, upper_bound, result)) {
          return false;
        }
      }
      return true;
    }


    template<typename T>
    bool validate_bounds(T value, const T &lower_bound, const T &upper_bound) {
      if (value > upper_bound || value < lower_bound) {
        return false;
      }
      return true;
    }

    template<typename T>
    bool validate_bounds(const std::string &name, T value, const T &lower_bound, const T &upper_bound,
                         rcl_interfaces::msg::SetParametersResult &result) {
      if (!validate_bounds(value, lower_bound, upper_bound)) {
        result.reason = std::string("Invalid value for parameter ") + name + ". Value not within the required bounds.";
        return false;
      }
      return true;
    }


    struct params {
      std::vector<std::string> joints_ = {"UNDEFINED"};
      std::vector<std::string> command_interfaces_ = {"UNDEFINED"};
      std::vector<std::string> state_interfaces_ = {"UNDEFINED"};
      std::vector<std::string> chainable_command_interfaces_ = {"UNDEFINED"};
      struct kinematics {
        std::string plugin_name_ = "UNDEFINED";
        std::string plugin_package_ = "UNDEFINED";
        std::string base_ = "UNDEFINED";
        std::string tip_ = "UNDEFINED";
        double alpha_ = 0.0005;
        std::string group_name_ = "UNDEFINED";
      } kinematics_;
      struct ft_sensor {
        std::string name_ = "UNDEFINED";
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
        double filter_coefficient_ = 0.005;
      } ft_sensor_;
      struct control {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
      } control_;
      struct fixed_world_frame {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
      } fixed_world_frame_;
      struct gravity_compensation {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
        struct CoG {
          std::vector<double> pos_ = {std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN()};
          double force_ = std::numeric_limits<double>::quiet_NaN();
        } CoG_;
      } gravity_compensation_;
      struct admittance {
        std::vector<bool> selected_axes_ = {false, false, false, false, false, false};
        std::vector<double> mass_ = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN()};
        std::vector<double> damping_ratio_ = {std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN()};
        std::vector<double> stiffness_ = {std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN()};
      } admittance_;
      bool enable_parameter_update_without_reactivation_ = true;
      bool use_feedforward_commanded_input_ = true;

    } params_;

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "joints" && validate_length("joints", param.as_string_array(), 1, result)) {
          params_.joints_ = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces" &&
            validate_length("command_interfaces", param.as_string_array(), 1, result)) {
          params_.command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces" &&
            validate_length("state_interfaces", param.as_string_array(), 1, result)) {
          params_.state_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces" &&
            validate_length("chainable_command_interfaces", param.as_string_array(), 1, result)) {
          params_.chainable_command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "kinematics.plugin_name") {
          params_.kinematics_.plugin_name_ = param.as_string();
        }
        if (param.get_name() == "kinematics.plugin_package") {
          params_.kinematics_.plugin_package_ = param.as_string();
        }
        if (param.get_name() == "kinematics.base") {
          params_.kinematics_.base_ = param.as_string();
        }
        if (param.get_name() == "kinematics.tip") {
          params_.kinematics_.tip_ = param.as_string();
        }
        if (param.get_name() == "kinematics.alpha") {
          params_.kinematics_.alpha_ = param.as_double();
        }
        if (param.get_name() == "kinematics.group_name") {
          params_.kinematics_.group_name_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.name") {
          params_.ft_sensor_.name_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.id") {
          params_.ft_sensor_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.external") {
          params_.ft_sensor_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "ft_sensor.filter_coefficient") {
          params_.ft_sensor_.filter_coefficient_ = param.as_double();
        }
        if (param.get_name() == "control.frame.id") {
          params_.control_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "control.frame.external") {
          params_.control_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "fixed_world_frame.frame.id") {
          params_.fixed_world_frame_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "fixed_world_frame.frame.external") {
          params_.fixed_world_frame_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.frame.id") {
          params_.gravity_compensation_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "gravity_compensation.frame.external") {
          params_.gravity_compensation_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.CoG.pos" &&
            validate_length("gravity_compensation.CoG.pos", param.as_double_array(), 3, result)) {
          params_.gravity_compensation_.CoG_.pos_ = param.as_double_array();
        }
        if (param.get_name() == "gravity_compensation.CoG.force") {
          params_.gravity_compensation_.CoG_.force_ = param.as_double();
        }
        if (param.get_name() == "admittance.selected_axes" &&
            validate_length("admittance.selected_axes", param.as_bool_array(), 6, result)) {
          params_.admittance_.selected_axes_ = param.as_bool_array();
        }
        if (param.get_name() == "admittance.mass" &&
            validate_length("admittance.mass", param.as_double_array(), 6, result) &&
            validate_bounds("admittance.mass", param.as_double_array(), 0.0001, 100000000.0, result)) {
          params_.admittance_.mass_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.damping_ratio" &&
            validate_length("admittance.damping_ratio", param.as_double_array(), 6, result) &&
            validate_bounds("admittance.damping_ratio", param.as_double_array(), 0.1, 10.0, result)) {
          params_.admittance_.damping_ratio_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.stiffness" &&
            validate_length("admittance.stiffness", param.as_double_array(), 6, result)) {
          params_.admittance_.stiffness_ = param.as_double_array();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          params_.enable_parameter_update_without_reactivation_ = param.as_bool();
        }
        if (param.get_name() == "use_feedforward_commanded_input") {
          params_.use_feedforward_commanded_input_ = param.as_bool();
        }

      }
      return result;
    }

    void declare_params(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {

      std::unordered_map<std::string, rcl_interfaces::msg::ParameterDescriptor> desc_map;

      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which joints will be used by the controller";
        descriptor.read_only = true;
        desc_map["joints"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which command interfaces to claim";
        descriptor.read_only = true;
        desc_map["command_interfaces"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which state interfaces to claim";
        descriptor.read_only = true;
        desc_map["state_interfaces"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which chainable interfaces to claim";
        descriptor.read_only = true;
        desc_map["chainable_command_interfaces"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which kinematics plugin to load";
        descriptor.read_only = false;
        desc_map["kinematics.plugin_name"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the package to load the kinematics plugin from";
        descriptor.read_only = false;
        desc_map["kinematics.plugin_package"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the base link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        desc_map["kinematics.base"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the end effector link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        desc_map["kinematics.tip"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the damping coefficient for the Jacobian pseudo inverse";
        descriptor.read_only = false;
        desc_map["kinematics.alpha"] = descriptor;
        if (!parameters_interface->has_parameter("kinematics.alpha")) {
          auto p_kinematics_alpha = rclcpp::ParameterValue(params_.kinematics_.alpha_);
          parameters_interface->declare_parameter("kinematics.alpha", p_kinematics_alpha, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the group name for planning with Moveit";
        descriptor.read_only = false;
        desc_map["kinematics.group_name"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "name of the force torque sensor in the robot description";
        descriptor.read_only = false;
        desc_map["ft_sensor.name"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame of the force torque sensor";
        descriptor.read_only = false;
        desc_map["ft_sensor.frame.id"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the force torque sensor is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        desc_map["ft_sensor.frame.external"] = descriptor;
        if (!parameters_interface->has_parameter("ft_sensor.frame.external")) {
          auto p_ft_sensor_frame_external = rclcpp::ParameterValue(params_.ft_sensor_.frame_.external_);
          parameters_interface->declare_parameter("ft_sensor.frame.external", p_ft_sensor_frame_external, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the coefficient for the sensor's exponential filter";
        descriptor.read_only = false;
        desc_map["ft_sensor.filter_coefficient"] = descriptor;
        if (!parameters_interface->has_parameter("ft_sensor.filter_coefficient")) {
          auto p_ft_sensor_filter_coefficient = rclcpp::ParameterValue(params_.ft_sensor_.filter_coefficient_);
          parameters_interface->declare_parameter("ft_sensor.filter_coefficient", p_ft_sensor_filter_coefficient,
                                                  descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "control frame used for admittance control";
        descriptor.read_only = false;
        desc_map["control.frame.id"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the control frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        desc_map["control.frame.external"] = descriptor;
        if (!parameters_interface->has_parameter("control.frame.external")) {
          auto p_control_frame_external = rclcpp::ParameterValue(params_.control_.frame_.external_);
          parameters_interface->declare_parameter("control.frame.external", p_control_frame_external, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "world frame, gravity points down (neg. Z) in this frame";
        descriptor.read_only = false;
        desc_map["fixed_world_frame.frame.id"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the world frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        desc_map["fixed_world_frame.frame.external"] = descriptor;
        if (!parameters_interface->has_parameter("fixed_world_frame.frame.external")) {
          auto p_fixed_world_frame_frame_external = rclcpp::ParameterValue(params_.fixed_world_frame_.frame_.external_);
          parameters_interface->declare_parameter("fixed_world_frame.frame.external",
                                                  p_fixed_world_frame_frame_external, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame which center of gravity (CoG) is defined in";
        descriptor.read_only = false;
        desc_map["gravity_compensation.frame.id"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the center of gravity frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        desc_map["gravity_compensation.frame.external"] = descriptor;
        if (!parameters_interface->has_parameter("gravity_compensation.frame.external")) {
          auto p_gravity_compensation_frame_external = rclcpp::ParameterValue(
              params_.gravity_compensation_.frame_.external_);
          parameters_interface->declare_parameter("gravity_compensation.frame.external",
                                                  p_gravity_compensation_frame_external, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "position of the center of gravity (CoG) in its frame";
        descriptor.read_only = false;
        desc_map["gravity_compensation.CoG.pos"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "weight of the end effector, e.g mass * 9.81";
        descriptor.read_only = false;
        desc_map["gravity_compensation.CoG.force"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the axes x, y, z, rx, ry, and rz are enabled";
        descriptor.read_only = false;
        desc_map["admittance.selected_axes"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies mass values for x, y, z, rx, ry, and rz used in the admittance calculation";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0001;
        range.to_value = 100000000.0;
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        desc_map["admittance.mass"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.1;
        range.to_value = 10.0;
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        desc_map["admittance.damping_ratio"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies stiffness values for x, y, z, rx, ry, and rz used in the admittance calculation";
        descriptor.read_only = false;
        desc_map["admittance.stiffness"] = descriptor;
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "if enabled, configurable parameters will be dynamically updated in the control loop";
        descriptor.read_only = false;
        desc_map["enable_parameter_update_without_reactivation"] = descriptor;
        if (!parameters_interface->has_parameter("enable_parameter_update_without_reactivation")) {
          auto p_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
              params_.enable_parameter_update_without_reactivation_);
          parameters_interface->declare_parameter("enable_parameter_update_without_reactivation",
                                                  p_enable_parameter_update_without_reactivation, descriptor);
        }
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "if enabled, the velocity commanded to the admittance controller is added to its calculated admittance velocity";
        descriptor.read_only = false;
        desc_map["use_feedforward_commanded_input"] = descriptor;
        if (!parameters_interface->has_parameter("use_feedforward_commanded_input")) {
          auto p_use_feedforward_commanded_input = rclcpp::ParameterValue(params_.use_feedforward_commanded_input_);
          parameters_interface->declare_parameter("use_feedforward_commanded_input", p_use_feedforward_commanded_input,
                                                  descriptor);
        }
      }


      std::map<std::string, rclcpp::ParameterValue> overrides = parameters_interface->get_parameter_overrides();
      declare_overrides(parameters_interface, overrides, desc_map);

      params_.joints_ = parameters_interface->get_parameter("joints").as_string_array();
      params_.command_interfaces_ = parameters_interface->get_parameter("command_interfaces").as_string_array();
      params_.state_interfaces_ = parameters_interface->get_parameter("state_interfaces").as_string_array();
      params_.chainable_command_interfaces_ = parameters_interface->get_parameter(
          "chainable_command_interfaces").as_string_array();
      params_.kinematics_.plugin_name_ = parameters_interface->get_parameter("kinematics.plugin_name").as_string();
      params_.kinematics_.plugin_package_ = parameters_interface->get_parameter(
          "kinematics.plugin_package").as_string();
      params_.kinematics_.base_ = parameters_interface->get_parameter("kinematics.base").as_string();
      params_.kinematics_.tip_ = parameters_interface->get_parameter("kinematics.tip").as_string();
      params_.kinematics_.alpha_ = parameters_interface->get_parameter("kinematics.alpha").as_double();
      params_.kinematics_.group_name_ = parameters_interface->get_parameter("kinematics.group_name").as_string();
      params_.ft_sensor_.name_ = parameters_interface->get_parameter("ft_sensor.name").as_string();
      params_.ft_sensor_.frame_.id_ = parameters_interface->get_parameter("ft_sensor.frame.id").as_string();
      params_.ft_sensor_.frame_.external_ = parameters_interface->get_parameter("ft_sensor.frame.external").as_bool();
      params_.ft_sensor_.filter_coefficient_ = parameters_interface->get_parameter(
          "ft_sensor.filter_coefficient").as_double();
      params_.control_.frame_.id_ = parameters_interface->get_parameter("control.frame.id").as_string();
      params_.control_.frame_.external_ = parameters_interface->get_parameter("control.frame.external").as_bool();
      params_.fixed_world_frame_.frame_.id_ = parameters_interface->get_parameter(
          "fixed_world_frame.frame.id").as_string();
      params_.fixed_world_frame_.frame_.external_ = parameters_interface->get_parameter(
          "fixed_world_frame.frame.external").as_bool();
      params_.gravity_compensation_.frame_.id_ = parameters_interface->get_parameter(
          "gravity_compensation.frame.id").as_string();
      params_.gravity_compensation_.frame_.external_ = parameters_interface->get_parameter(
          "gravity_compensation.frame.external").as_bool();
      if (!validate_length(parameters_interface->get_parameter("gravity_compensation.CoG.pos").as_double_array(), 3)) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid value set during initialization for parameter gravity_compensation.CoG.pos ");
      }
      params_.gravity_compensation_.CoG_.pos_ = parameters_interface->get_parameter(
          "gravity_compensation.CoG.pos").as_double_array();
      params_.gravity_compensation_.CoG_.force_ = parameters_interface->get_parameter(
          "gravity_compensation.CoG.force").as_double();
      if (!validate_length(parameters_interface->get_parameter("admittance.selected_axes").as_bool_array(), 6)) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid value set during initialization for parameter admittance.selected_axes ");
      }
      params_.admittance_.selected_axes_ = parameters_interface->get_parameter(
          "admittance.selected_axes").as_bool_array();
      if (!validate_length(parameters_interface->get_parameter("admittance.mass").as_double_array(), 6) ||
          !validate_bounds(parameters_interface->get_parameter("admittance.mass").as_double_array(), 0.0001,
                           100000000.0)) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid value set during initialization for parameter admittance.mass ");
      }
      params_.admittance_.mass_ = parameters_interface->get_parameter("admittance.mass").as_double_array();
      if (!validate_length(parameters_interface->get_parameter("admittance.damping_ratio").as_double_array(), 6) ||
          !validate_bounds(parameters_interface->get_parameter("admittance.damping_ratio").as_double_array(), 0.1,
                           10.0)) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid value set during initialization for parameter admittance.damping_ratio ");
      }
      params_.admittance_.damping_ratio_ = parameters_interface->get_parameter(
          "admittance.damping_ratio").as_double_array();
      if (!validate_length(parameters_interface->get_parameter("admittance.stiffness").as_double_array(), 6)) {
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Invalid value set during initialization for parameter admittance.stiffness ");
      }
      params_.admittance_.stiffness_ = parameters_interface->get_parameter("admittance.stiffness").as_double_array();
      params_.enable_parameter_update_without_reactivation_ = parameters_interface->get_parameter(
          "enable_parameter_update_without_reactivation").as_bool();
      params_.use_feedforward_commanded_input_ = parameters_interface->get_parameter(
          "use_feedforward_commanded_input").as_bool();

    }
  };

} // namespace admittance_struct_parameters
