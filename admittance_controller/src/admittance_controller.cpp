// Copyright (c) 2021, PickNik, Inc.
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

#include <functional>
#include <limits>
#include <string>
#include <vector>

// #include "Eigen/Core"

#include "admittance_controller/admittance_controller.hpp"
#include "admittance_controller/incremental_kinematics.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace admittance_controller
{
AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::return_type AdmittanceController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    get_node()->declare_parameter<std::vector<std::string>>("joints", {});
    get_node()->declare_parameter<std::vector<std::string>>("command_interfaces", {});
    get_node()->declare_parameter<std::vector<std::string>>("state_interfaces", {});
    get_node()->declare_parameter<std::string>("ft_sensor_name", "");

    get_node()->declare_parameter<std::string>("IK.base", "");
    get_node()->declare_parameter<std::string>("IK.tip", "");
    // TODO(destogl): enable when IK-plugin support is added
//     get_node()->declare_parameter<std::string>("IK.plugin", "");
    get_node()->declare_parameter<std::string>("IK.group_name", "");

    get_node()->declare_parameter<std::string>("control_frame", "");
    get_node()->declare_parameter<std::string>("endeffector_frame", "");
    get_node()->declare_parameter<std::string>("fixed_world_frame", "");
    get_node()->declare_parameter<std::string>("sensor_frame", "");

    get_node()->declare_parameter<std::vector<double>>("gravity_compensation.masses", {});
    get_node()->declare_parameter<std::vector<double>>("gravity_compensation.center_of_masses", {});

    // TODO(destogl): enable when force/position control is implemented
//     get_node()->declare_parameter<bool>("admitance.unified_mode", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.x", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.y", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.z", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.rx", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.ry", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.rz", false);

    get_node()->declare_parameter<double>("admittance.mass.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.rz", std::numeric_limits<double>::quiet_NaN());

    get_node()->declare_parameter<double>("admittance.damping.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.rz", std::numeric_limits<double>::quiet_NaN());

    get_node()->declare_parameter<double>("admittance.stiffness.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.rz", std::numeric_limits<double>::quiet_NaN());

//     get_node()->declare_parameter<std::vector<double>>("")
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();

  return controller_interface::return_type::OK;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty = [&](
    std::vector<std::string> & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string_array();
    return error_if_empty(parameter, parameter_name);
  };

  auto get_string_param_and_error_if_empty = [&](
    std::string & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  // TODO(destogl): If we would use C++20 than we can use templates here
  auto get_bool_param_and_error_if_empty = [&](
    bool & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<bool>();
    return false; // TODO(destogl): how to check "if_empty" for bool?
  };

  auto get_double_param_and_error_if_empty = [&](
    double & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<double>();
    if (std::isnan(parameter)) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was not set", parameter_name);
      return true;
    }
    return false;
  };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
    get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
    get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
    get_string_param_and_error_if_empty(admittance_->ik_base_frame_, "IK.base") ||
    get_string_param_and_error_if_empty(admittance_->ik_tip_frame_, "IK.tip") ||
    get_string_param_and_error_if_empty(admittance_->ik_group_name_, "IK.group_name") ||
    get_string_param_and_error_if_empty(admittance_->control_frame_, "control_frame") ||
    get_string_param_and_error_if_empty(admittance_->endeffector_frame_, "endeffector_frame") ||
    get_string_param_and_error_if_empty(admittance_->fixed_world_frame_, "fixed_world_frame") ||
    get_string_param_and_error_if_empty(admittance_->sensor_frame_, "sensor_frame") ||
    // TODO(destogl): add unified mode considering target force
//     get_bool_param_and_error_if_empty(unified_mode_, "admittance.unified_mode") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[0], "admittance.selected_axes.x") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[1], "admittance.selected_axes.y") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[2], "admittance.selected_axes.z") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[3], "admittance.selected_axes.rx") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[4], "admittance.selected_axes.ry") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[5], "admittance.selected_axes.rz") ||

    get_double_param_and_error_if_empty(admittance_->mass_[0], "admittance.mass.x") ||
    get_double_param_and_error_if_empty(admittance_->mass_[1], "admittance.mass.y") ||
    get_double_param_and_error_if_empty(admittance_->mass_[2], "admittance.mass.z") ||
    get_double_param_and_error_if_empty(admittance_->mass_[3], "admittance.mass.rx") ||
    get_double_param_and_error_if_empty(admittance_->mass_[4], "admittance.mass.ry") ||
    get_double_param_and_error_if_empty(admittance_->mass_[5], "admittance.mass.rz") ||

    get_double_param_and_error_if_empty(admittance_->damping_[0], "admittance.damping.x") ||
    get_double_param_and_error_if_empty(admittance_->damping_[1], "admittance.damping.y") ||
    get_double_param_and_error_if_empty(admittance_->damping_[2], "admittance.damping.z") ||
    get_double_param_and_error_if_empty(admittance_->damping_[3], "admittance.damping.rx") ||
    get_double_param_and_error_if_empty(admittance_->damping_[4], "admittance.damping.ry") ||
    get_double_param_and_error_if_empty(admittance_->damping_[5], "admittance.damping.rz") ||

    get_double_param_and_error_if_empty(admittance_->stiffness_[0], "admittance.stiffness.x") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[1], "admittance.stiffness.y") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[2], "admittance.stiffness.z") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[3], "admittance.stiffness.rx") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[4], "admittance.stiffness.ry") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[5], "admittance.stiffness.rz")
    )
  {
    return CallbackReturn::ERROR;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface type '" + interface + "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_POSITION)) {
    has_position_command_interface_ = true;
  }
  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_command_interface_ = true;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "State interface type '" + interface + "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (!controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(get_node()->get_logger(), "State interface type '" + std::string(hardware_interface::HW_IF_POSITION) + "' has to be always present allowed!");
    return CallbackReturn::ERROR;
  }

  if (controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_state_interface_ = true;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types) {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index) {
      if (index != 0) {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
              get_interface_list(state_interface_types_).c_str());

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(ft_sensor_name_));

  // Subscribers and callbacks
  auto callback_input_force = [&](const std::shared_ptr<ControllerCommandForceMsg> msg)
    -> void
    {
      input_force_command_.writeFromNonRT(msg);
    };
  input_force_command_subscriber_ = get_node()->create_subscription<ControllerCommandForceMsg>(
    "~/force_commands", rclcpp::SystemDefaultsQoS(), callback_input_force);

  auto callback_input_pose = [&](const std::shared_ptr<ControllerCommandPoseMsg> msg)
  -> void
  {
    input_pose_command_.writeFromNonRT(msg);
  };
  input_pose_command_subscriber_ = get_node()->create_subscription<ControllerCommandPoseMsg>(
    "~/pose_commands", rclcpp::SystemDefaultsQoS(), callback_input_pose);

  // TODO(destogl): Add subscriber for velocity scaling

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  // Configure AdmittanceRule
  admittance_->configure(get_node());

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : joint_names_) {
    for (const auto & interface : command_interface_types_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(6 + joint_names_.size() * state_interface_types_.size());

  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();

  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interface_types_) {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // order all joints in the storage
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %u '%s' command interfaces, got %u.",
                   joint_names_.size(), interface.c_str(), joint_command_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %u '%s' state interfaces, got %u.",
                   joint_names_.size(), interface.c_str(), joint_state_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Initialize interface of the FTS semantic semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize Admittance Rule from current states
  admittance_->reset();
  previous_time_ = get_node()->now();

  // Set initial command values
  std::shared_ptr<ControllerCommandForceMsg> msg_force = std::make_shared<ControllerCommandForceMsg>();
  msg_force->header.frame_id = admittance_->control_frame_;
  input_force_command_.writeFromNonRT(msg_force);

  std::shared_ptr<ControllerCommandPoseMsg> msg_pose = std::make_shared<ControllerCommandPoseMsg>();
  msg_pose->header.frame_id = admittance_->control_frame_;
  input_pose_command_.writeFromNonRT(msg_pose);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[0][index].get().get_value());
  }

  for (auto index = 0ul; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  force_torque_sensor_->release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update()
{
  // get input commands
  // TODO(destogl): Enable this when unified mode is used
  auto input_force_cmd = input_force_command_.readFromRT();
  auto input_pose_cmd = input_pose_command_.readFromRT();

  // Position has to always there
  auto num_joints = joint_state_interface_[0].size();
  std::vector<double> current_joint_states(num_joints);

  for (auto i = 0u; i < num_joints; ++i) {
    current_joint_states.emplace_back(joint_state_interface_[0][i].get().get_value());
  }

  // TODO(destogl): Enable this when unified mode is used
//   if (admittance_.unified_mode_) {
//     admittance_->update(current_joint_states, force_torque_sensor_->get_values_as_message(), **input_pose_cmd, **input_force_cmd, get_node()->now() - previous_time_);
//   } else {
  admittance_->update(current_joint_states, force_torque_sensor_->get_values_as_message(), **input_pose_cmd, get_node()->now() - previous_time_);
//   }
  previous_time_ = get_node()->now();

  // Write new joint angles to the robot

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.input_force_command = **input_force_cmd;
  state_publisher_->msg_.input_pose_command = **input_pose_cmd;
  admittance_->get_controller_state(state_publisher_->msg_);
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController,
  controller_interface::ControllerInterface)
