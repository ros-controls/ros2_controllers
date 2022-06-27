// this is auto-generated code 

#include <rclcpp/node.hpp>
#include <vector>
#include <string>


namespace admittance_struct_parameters {

  struct admittance_struct {
    // if true, prevent parameters from updating
    bool lock_params_ = false;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;

    admittance_struct(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      declare_params(parameters_interface);
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters) {
        return this->update(parameters);
      };
      handle_ = parameters_interface->add_on_set_parameters_callback(update_param_cb);
    }

    struct params {
      std::vector<std::string> joints_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                          "wrist_2_joint", "wrist_3_joint"};
      std::vector<std::string> command_interfaces_ = {"position"};
      std::vector<std::string> state_interfaces_ = {"position", "velocity"};
      std::vector<std::string> chainable_command_interfaces_ = {"position", "velocity"};
      struct kinematics {
        std::string plugin_name_ = "kdl_plugin/KDLKinematics";
        std::string base_ = "base_link";
        std::string tip_ = "ee_link";
        std::string group_name_ = "ur5e_manipulator";
        double alpha_ = 5e-06;
      } kinematics_;
      struct ft_sensor {
        std::string name_ = "tcp_fts_sensor";
        struct frame {
          std::string id_ = "ee_link";
          bool external_ = false;
        } frame_;
        double filter_coefficient_ = 0.005;
      } ft_sensor_;
      struct control {
        struct frame {
          std::string id_ = "ee_link";
          bool external_ = false;
        } frame_;
      } control_;
      struct fixed_world_frame {
        std::string id_ = "base_link";
        bool external_ = false;
      } fixed_world_frame_;
      struct gravity_compensation {
        struct frame {
          std::string id_ = "ee_link";
          bool external_ = false;
        } frame_;
        struct CoG {
          std::vector<double> pos_ = {0.1, 0.0, 0.0};
          double force_ = 23.0;
        } CoG_;
      } gravity_compensation_;
      struct admittance {
        std::vector<bool> selected_axes_ = {true, true, true, true, true, true};
        std::vector<double> mass_ = {3.0, 3.0, 3.0, 0.05, 0.05, 0.05};
        std::vector<double> damping_ratio_ = {2.828427, 2.828427, 2.828427, 2.828427, 2.828427, 2.828427};
        std::vector<double> stiffness_ = {50.0, 50.0, 50.0, 1.0, 1.0, 1.0};
      } admittance_;
      bool enable_parameter_update_without_reactivation_ = true;
      bool use_feedforward_commanded_input_ = true;
      std::string joint_limiter_type_ = "joint_limits/SimpleJointLimiter";
      double state_publish_rate_ = 200.0;

    } params_;

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !lock_params_;
      if (lock_params_) {
        result.reason = "The parameters can not be updated because they are currently locked.";
        return result;
      }

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "joints") {
          params_.joints_ = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces") {
          params_.command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces") {
          params_.state_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces") {
          params_.chainable_command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "kinematics.plugin_name") {
          params_.kinematics_.plugin_name_ = param.as_string();
        }
        if (param.get_name() == "kinematics.base") {
          params_.kinematics_.base_ = param.as_string();
        }
        if (param.get_name() == "kinematics.tip") {
          params_.kinematics_.tip_ = param.as_string();
        }
        if (param.get_name() == "kinematics.group_name") {
          params_.kinematics_.group_name_ = param.as_string();
        }
        if (param.get_name() == "kinematics.alpha") {
          params_.kinematics_.alpha_ = param.as_double();
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
        if (param.get_name() == "fixed_world_frame.id") {
          params_.fixed_world_frame_.id_ = param.as_string();
        }
        if (param.get_name() == "fixed_world_frame.external") {
          params_.fixed_world_frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.frame.id") {
          params_.gravity_compensation_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "gravity_compensation.frame.external") {
          params_.gravity_compensation_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.CoG.pos") {
          params_.gravity_compensation_.CoG_.pos_ = param.as_double_array();
        }
        if (param.get_name() == "gravity_compensation.CoG.force") {
          params_.gravity_compensation_.CoG_.force_ = param.as_double();
        }
        if (param.get_name() == "admittance.selected_axes") {
          params_.admittance_.selected_axes_ = param.as_bool_array();
        }
        if (param.get_name() == "admittance.mass") {
          params_.admittance_.mass_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.damping_ratio") {
          params_.admittance_.damping_ratio_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.stiffness") {
          params_.admittance_.stiffness_ = param.as_double_array();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          params_.enable_parameter_update_without_reactivation_ = param.as_bool();
        }
        if (param.get_name() == "use_feedforward_commanded_input") {
          params_.use_feedforward_commanded_input_ = param.as_bool();
        }
        if (param.get_name() == "joint_limiter_type") {
          params_.joint_limiter_type_ = param.as_string();
        }
        if (param.get_name() == "state_publish_rate") {
          params_.state_publish_rate_ = param.as_double();
        }

      }
      return result;
    }

    void declare_params(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      if (!parameters_interface->has_parameter("joints")) {
        auto p_joints = rclcpp::ParameterValue(params_.joints_);
        parameters_interface->declare_parameter("joints", p_joints);
      } else {
        params_.joints_ = parameters_interface->get_parameter("joints").as_string_array();
      }
      if (!parameters_interface->has_parameter("command_interfaces")) {
        auto p_command_interfaces = rclcpp::ParameterValue(params_.command_interfaces_);
        parameters_interface->declare_parameter("command_interfaces", p_command_interfaces);
      } else {
        params_.command_interfaces_ = parameters_interface->get_parameter("command_interfaces").as_string_array();
      }
      if (!parameters_interface->has_parameter("state_interfaces")) {
        auto p_state_interfaces = rclcpp::ParameterValue(params_.state_interfaces_);
        parameters_interface->declare_parameter("state_interfaces", p_state_interfaces);
      } else {
        params_.state_interfaces_ = parameters_interface->get_parameter("state_interfaces").as_string_array();
      }
      if (!parameters_interface->has_parameter("chainable_command_interfaces")) {
        auto p_chainable_command_interfaces = rclcpp::ParameterValue(params_.chainable_command_interfaces_);
        parameters_interface->declare_parameter("chainable_command_interfaces", p_chainable_command_interfaces);
      } else {
        params_.chainable_command_interfaces_ = parameters_interface->get_parameter(
            "chainable_command_interfaces").as_string_array();
      }
      if (!parameters_interface->has_parameter("kinematics.plugin_name")) {
        auto p_kinematics_plugin_name = rclcpp::ParameterValue(params_.kinematics_.plugin_name_);
        parameters_interface->declare_parameter("kinematics.plugin_name", p_kinematics_plugin_name);
      } else {
        params_.kinematics_.plugin_name_ = parameters_interface->get_parameter("kinematics.plugin_name").as_string();
      }
      if (!parameters_interface->has_parameter("kinematics.base")) {
        auto p_kinematics_base = rclcpp::ParameterValue(params_.kinematics_.base_);
        parameters_interface->declare_parameter("kinematics.base", p_kinematics_base);
      } else {
        params_.kinematics_.base_ = parameters_interface->get_parameter("kinematics.base").as_string();
      }
      if (!parameters_interface->has_parameter("kinematics.tip")) {
        auto p_kinematics_tip = rclcpp::ParameterValue(params_.kinematics_.tip_);
        parameters_interface->declare_parameter("kinematics.tip", p_kinematics_tip);
      } else {
        params_.kinematics_.tip_ = parameters_interface->get_parameter("kinematics.tip").as_string();
      }
      if (!parameters_interface->has_parameter("kinematics.group_name")) {
        auto p_kinematics_group_name = rclcpp::ParameterValue(params_.kinematics_.group_name_);
        parameters_interface->declare_parameter("kinematics.group_name", p_kinematics_group_name);
      } else {
        params_.kinematics_.group_name_ = parameters_interface->get_parameter("kinematics.group_name").as_string();
      }
      if (!parameters_interface->has_parameter("kinematics.alpha")) {
        auto p_kinematics_alpha = rclcpp::ParameterValue(params_.kinematics_.alpha_);
        parameters_interface->declare_parameter("kinematics.alpha", p_kinematics_alpha);
      } else {
        params_.kinematics_.alpha_ = parameters_interface->get_parameter("kinematics.alpha").as_double();
      }
      if (!parameters_interface->has_parameter("ft_sensor.name")) {
        auto p_ft_sensor_name = rclcpp::ParameterValue(params_.ft_sensor_.name_);
        parameters_interface->declare_parameter("ft_sensor.name", p_ft_sensor_name);
      } else {
        params_.ft_sensor_.name_ = parameters_interface->get_parameter("ft_sensor.name").as_string();
      }
      if (!parameters_interface->has_parameter("ft_sensor.frame.id")) {
        auto p_ft_sensor_frame_id = rclcpp::ParameterValue(params_.ft_sensor_.frame_.id_);
        parameters_interface->declare_parameter("ft_sensor.frame.id", p_ft_sensor_frame_id);
      } else {
        params_.ft_sensor_.frame_.id_ = parameters_interface->get_parameter("ft_sensor.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("ft_sensor.frame.external")) {
        auto p_ft_sensor_frame_external = rclcpp::ParameterValue(params_.ft_sensor_.frame_.external_);
        parameters_interface->declare_parameter("ft_sensor.frame.external", p_ft_sensor_frame_external);
      } else {
        params_.ft_sensor_.frame_.external_ = parameters_interface->get_parameter("ft_sensor.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("ft_sensor.filter_coefficient")) {
        auto p_ft_sensor_filter_coefficient = rclcpp::ParameterValue(params_.ft_sensor_.filter_coefficient_);
        parameters_interface->declare_parameter("ft_sensor.filter_coefficient", p_ft_sensor_filter_coefficient);
      } else {
        params_.ft_sensor_.filter_coefficient_ = parameters_interface->get_parameter(
            "ft_sensor.filter_coefficient").as_double();
      }
      if (!parameters_interface->has_parameter("control.frame.id")) {
        auto p_control_frame_id = rclcpp::ParameterValue(params_.control_.frame_.id_);
        parameters_interface->declare_parameter("control.frame.id", p_control_frame_id);
      } else {
        params_.control_.frame_.id_ = parameters_interface->get_parameter("control.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("control.frame.external")) {
        auto p_control_frame_external = rclcpp::ParameterValue(params_.control_.frame_.external_);
        parameters_interface->declare_parameter("control.frame.external", p_control_frame_external);
      } else {
        params_.control_.frame_.external_ = parameters_interface->get_parameter("control.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("fixed_world_frame.id")) {
        auto p_fixed_world_frame_id = rclcpp::ParameterValue(params_.fixed_world_frame_.id_);
        parameters_interface->declare_parameter("fixed_world_frame.id", p_fixed_world_frame_id);
      } else {
        params_.fixed_world_frame_.id_ = parameters_interface->get_parameter("fixed_world_frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("fixed_world_frame.external")) {
        auto p_fixed_world_frame_external = rclcpp::ParameterValue(params_.fixed_world_frame_.external_);
        parameters_interface->declare_parameter("fixed_world_frame.external", p_fixed_world_frame_external);
      } else {
        params_.fixed_world_frame_.external_ = parameters_interface->get_parameter(
            "fixed_world_frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("gravity_compensation.frame.id")) {
        auto p_gravity_compensation_frame_id = rclcpp::ParameterValue(params_.gravity_compensation_.frame_.id_);
        parameters_interface->declare_parameter("gravity_compensation.frame.id", p_gravity_compensation_frame_id);
      } else {
        params_.gravity_compensation_.frame_.id_ = parameters_interface->get_parameter(
            "gravity_compensation.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("gravity_compensation.frame.external")) {
        auto p_gravity_compensation_frame_external = rclcpp::ParameterValue(
            params_.gravity_compensation_.frame_.external_);
        parameters_interface->declare_parameter("gravity_compensation.frame.external",
                                                p_gravity_compensation_frame_external);
      } else {
        params_.gravity_compensation_.frame_.external_ = parameters_interface->get_parameter(
            "gravity_compensation.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.pos")) {
        auto p_gravity_compensation_CoG_pos = rclcpp::ParameterValue(params_.gravity_compensation_.CoG_.pos_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.pos", p_gravity_compensation_CoG_pos);
      } else {
        params_.gravity_compensation_.CoG_.pos_ = parameters_interface->get_parameter(
            "gravity_compensation.CoG.pos").as_double_array();
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.force")) {
        auto p_gravity_compensation_CoG_force = rclcpp::ParameterValue(params_.gravity_compensation_.CoG_.force_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.force", p_gravity_compensation_CoG_force);
      } else {
        params_.gravity_compensation_.CoG_.force_ = parameters_interface->get_parameter(
            "gravity_compensation.CoG.force").as_double();
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes")) {
        auto p_admittance_selected_axes = rclcpp::ParameterValue(params_.admittance_.selected_axes_);
        parameters_interface->declare_parameter("admittance.selected_axes", p_admittance_selected_axes);
      } else {
        params_.admittance_.selected_axes_ = parameters_interface->get_parameter(
            "admittance.selected_axes").as_bool_array();
      }
      if (!parameters_interface->has_parameter("admittance.mass")) {
        auto p_admittance_mass = rclcpp::ParameterValue(params_.admittance_.mass_);
        parameters_interface->declare_parameter("admittance.mass", p_admittance_mass);
      } else {
        params_.admittance_.mass_ = parameters_interface->get_parameter("admittance.mass").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio")) {
        auto p_admittance_damping_ratio = rclcpp::ParameterValue(params_.admittance_.damping_ratio_);
        parameters_interface->declare_parameter("admittance.damping_ratio", p_admittance_damping_ratio);
      } else {
        params_.admittance_.damping_ratio_ = parameters_interface->get_parameter(
            "admittance.damping_ratio").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance.stiffness")) {
        auto p_admittance_stiffness = rclcpp::ParameterValue(params_.admittance_.stiffness_);
        parameters_interface->declare_parameter("admittance.stiffness", p_admittance_stiffness);
      } else {
        params_.admittance_.stiffness_ = parameters_interface->get_parameter("admittance.stiffness").as_double_array();
      }
      if (!parameters_interface->has_parameter("enable_parameter_update_without_reactivation")) {
        auto p_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
            params_.enable_parameter_update_without_reactivation_);
        parameters_interface->declare_parameter("enable_parameter_update_without_reactivation",
                                                p_enable_parameter_update_without_reactivation);
      } else {
        params_.enable_parameter_update_without_reactivation_ = parameters_interface->get_parameter(
            "enable_parameter_update_without_reactivation").as_bool();
      }
      if (!parameters_interface->has_parameter("use_feedforward_commanded_input")) {
        auto p_use_feedforward_commanded_input = rclcpp::ParameterValue(params_.use_feedforward_commanded_input_);
        parameters_interface->declare_parameter("use_feedforward_commanded_input", p_use_feedforward_commanded_input);
      } else {
        params_.use_feedforward_commanded_input_ = parameters_interface->get_parameter(
            "use_feedforward_commanded_input").as_bool();
      }
      if (!parameters_interface->has_parameter("joint_limiter_type")) {
        auto p_joint_limiter_type = rclcpp::ParameterValue(params_.joint_limiter_type_);
        parameters_interface->declare_parameter("joint_limiter_type", p_joint_limiter_type);
      } else {
        params_.joint_limiter_type_ = parameters_interface->get_parameter("joint_limiter_type").as_string();
      }
      if (!parameters_interface->has_parameter("state_publish_rate")) {
        auto p_state_publish_rate = rclcpp::ParameterValue(params_.state_publish_rate_);
        parameters_interface->declare_parameter("state_publish_rate", p_state_publish_rate);
      } else {
        params_.state_publish_rate_ = parameters_interface->get_parameter("state_publish_rate").as_double();
      }

    }
  };

} // namespace admittance_struct_parameters
