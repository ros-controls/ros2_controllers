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
/// \authors: Denis Stogl, Andy Zelenak

#include "admittance_controller/admittance_controller.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller
{
AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface()
{
}

CallbackReturn AdmittanceController::on_init()
{

  admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();

  try {
    // TODO: use variables as parameters
    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>({}));
    get_node()->declare_parameter<std::vector<std::string>>("command_interfaces", std::vector<std::string>({}));
    get_node()->declare_parameter<std::vector<std::string>>("state_interfaces", std::vector<std::string>({}));
    get_node()->declare_parameter<std::string>("ft_sensor_name", "");
    get_node()->declare_parameter<bool>("use_joint_commands_as_input", false);
    get_node()->declare_parameter<std::string>("joint_limiter_type", "joint_limits/SimpleJointLimiter");

    // TODO(destogl): enable when IK-plugin support is added
    // get_node()->declare_parameter<std::string>("IK.plugin", "");

    admittance_->parameters_.declare_parameters(get_node());

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
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

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
    get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
    get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
    get_bool_param_and_error_if_empty(use_joint_commands_as_input_, "use_joint_commands_as_input") ||
    get_string_param_and_error_if_empty(joint_limiter_type_, "joint_limiter_type") ||

    !admittance_->parameters_.get_parameters(get_node())
    )
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error happened during reading parameters");
    return CallbackReturn::ERROR;
  }

  try {
    admittance_->filter_chain_ =
    std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during filter chain creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  if (!admittance_->filter_chain_->configure("input_wrench_filter_chain",
    get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Could not configure sensor filter chain, please check if the "
                 "parameters are provided correctly.");
    return CallbackReturn::ERROR;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Command interface type '" << interface << "' not allowed!");
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
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << interface << "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (!controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << std::string(hardware_interface::HW_IF_POSITION) << "' has to be always present allowed!");
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

  auto num_joints = joint_names_.size();

  // Initialize joint limits
  if (!joint_limiter_type_.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Using joint limiter plugin: '%s'", joint_limiter_type_.c_str());
    joint_limiter_loader_ = std::make_shared<pluginlib::ClassLoader<JointLimiter>>(
      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>");
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_->createUnmanagedInstance(joint_limiter_type_));
    joint_limiter_->init(joint_names_, get_node());
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Not using joint limiter plugin as none defined.");
  }

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(ft_sensor_name_));

  // Subscribers and callbacks
  if (admittance_->unified_mode_) {
    auto callback_input_force = [&](const std::shared_ptr<ControllerCommandWrenchMsg> msg)
      -> void
      {
        input_wrench_command_.writeFromNonRT(msg);
      };
    input_wrench_command_subscriber_ = get_node()->create_subscription<ControllerCommandWrenchMsg>(
      "~/force_commands", rclcpp::SystemDefaultsQoS(), callback_input_force);
  }

  auto callback_input_joint = [&](const std::shared_ptr<ControllerCommandJointMsg> msg)
  -> void
  {
    input_joint_command_.writeFromNonRT(msg);
  };
  input_joint_command_subscriber_ = get_node()->create_subscription<ControllerCommandJointMsg>(
    "~/joint_commands", rclcpp::SystemDefaultsQoS(), callback_input_joint);

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

  // Initialize state message
  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.actual_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.desired_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.error_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->unlock();

  last_commanded_state_.positions.resize(num_joints);
  last_commanded_state_.velocities.resize(num_joints, 0.0);
  last_commanded_state_.accelerations.resize(num_joints, 0.0);

  if (use_joint_commands_as_input_) {
    RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Using Cartesian input mode.");
  }

  // Configure AdmittanceRule
  admittance_->configure(get_node());

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {

      return admittance_->parameters_.set_parameter_callback(parameters);
    });

  // HACK: This is workaround because it seems that updating parameters only in `on_activate` does
  // not work properly
  if (!admittance_->parameters_.check_if_parameters_are_valid()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Parameters are not valid and therefore will not be udpated");
  } else {
    admittance_->parameters_.update();
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
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
  const auto num_joints = joint_names_.size();

  // Update dynamic parameters before controller is started
  if (!admittance_->parameters_.check_if_parameters_are_valid()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Parameters are not valid and therefore will not be udpated");
  } else {
    admittance_->parameters_.update();
  }

  // order all joints in the storage
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_command_interface_[index].size());
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
        node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_state_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }


  // Initialize interface of the FTS semantic semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize Admittance Rule from current states
  admittance_->reset();

  read_state_from_hardware(last_commanded_state_);
  if (joint_limiter_)
  {
    joint_limiter_->configure(last_commanded_state_);
  }
  // Handle restart of controller by reading last_commanded_state_ from commands if not nan
  read_state_from_command_interfaces(last_commanded_state_);

  // Set initial command values - initialize all to simplify update
  std::shared_ptr<ControllerCommandWrenchMsg> msg_wrench = std::make_shared<ControllerCommandWrenchMsg>();
  msg_wrench->header.frame_id = admittance_->parameters_.control_frame_;
  input_wrench_command_.writeFromNonRT(msg_wrench);

  std::shared_ptr<ControllerCommandJointMsg> msg_joint = std::make_shared<ControllerCommandJointMsg>();
  msg_joint->joint_names = joint_names_;
  msg_joint->points.reserve(1);

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.reserve(num_joints);
  trajectory_point.velocities.resize(num_joints, 0.0);
  // FIXME(destogl): ATTENTION: This does not work properly, so using velocity mode and commenting positions out!
//   for (auto index = 0u; index < num_joints; ++index) {
//     trajectory_point.positions.emplace_back(joint_state_interface_[0][index].get().get_value());
//   }
  msg_joint->points.emplace_back(trajectory_point);

  input_joint_command_.writeFromNonRT(msg_joint);

  // TODO(destogl): Move this to the Cartesian admittance controller
  std::shared_ptr<ControllerCommandPoseMsg> msg_pose = std::make_shared<ControllerCommandPoseMsg>();
  msg_pose->header.frame_id = admittance_->parameters_.control_frame_;
  if (admittance_->get_pose_of_control_frame_in_base_frame(*msg_pose) !=
      controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Can not find transform from '%s' to '%s' needed in the update loop",
                 admittance_->parameters_.ik_base_frame_.c_str(), admittance_->parameters_.control_frame_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
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

controller_interface::return_type AdmittanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // get input commands
  auto input_wrench_cmd = input_wrench_command_.readFromRT();
  auto input_joint_cmd = input_joint_command_.readFromRT();
  auto input_pose_cmd = input_pose_command_.readFromRT();

  // Position has to always be there
  auto num_joints = joint_state_interface_[0].size();
  trajectory_msgs::msg::JointTrajectoryPoint current_joint_states;
  current_joint_states.positions.resize(num_joints);
  current_joint_states.velocities.resize(num_joints, 0.0);
  current_joint_states.accelerations.resize(num_joints, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint desired_joint_states;
  desired_joint_states.positions.resize(num_joints);
  desired_joint_states.velocities.resize(num_joints);
  desired_joint_states.accelerations.resize(num_joints);

  read_state_from_hardware(current_joint_states);

  if (admittance_->parameters_.open_loop_control_) {
    // TODO(destogl): This may not work in every case.
    // Please add checking which states are available and which not!
    current_joint_states = last_commanded_state_;
  }

  geometry_msgs::msg::Wrench ft_values;
  force_torque_sensor_->get_values_as_message(ft_values);

  // TODO(destogl): Enable this when unified mode is used
//   if (admittance_->unified_mode_) {
  //     admittance_->update(current_joint_states, ft_values, **input_pose_cmd, **input_wrench_cmd, period, desired_joint_states);
//   } else {

  // TODO(destogl): refactor this into different admittance controllers: 1. Pose input, Joint State input and Unified mode (is there need for switching between unified and non-unified mode?)
  if (use_joint_commands_as_input_) {
    std::array<double, 6> joint_deltas;
    // If there are no positions, expect velocities
    // TODO(destogl): add error handling
    if ((*input_joint_cmd)->points[0].positions.empty()) {
      for (auto index = 0u; index < num_joints; ++index) {
        joint_deltas[index] = (*input_joint_cmd)->points[0].velocities[index] * period.seconds();
      }
    } else {
      for (auto index = 0u; index < num_joints; ++index) {
        // TODO(destogl): ATTENTION: This does not work properly, deltas are getting neutralized and robot is not moving on external forces
        // TODO(anyone): Is here OK to use shortest_angular_distance?
        joint_deltas[index] = angles::shortest_angular_distance(current_joint_states.positions[index], (*input_joint_cmd)->points[0].positions[index]);
      }
    }

    admittance_->update(current_joint_states, ft_values, joint_deltas, period, desired_joint_states);
  } else {
    admittance_->update(current_joint_states, ft_values, **input_pose_cmd, period, desired_joint_states);
  }
//   }

  if (joint_limiter_)
  {
    joint_limiter_->enforce(current_joint_states, desired_joint_states, period);
  }

  // Write new joint angles to the robot
  for (auto index = 0u; index < num_joints; ++index) {
    if (has_position_command_interface_) {
      joint_command_interface_[0][index].get().set_value(desired_joint_states.positions[index]);
    }
    if (has_velocity_command_interface_) {
      joint_command_interface_[1][index].get().set_value(desired_joint_states.velocities[index]);
    }
  }
  last_commanded_state_ = desired_joint_states;

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.input_wrench_command = **input_wrench_cmd;
  state_publisher_->msg_.input_pose_command = **input_pose_cmd;
  state_publisher_->msg_.input_joint_command = **input_joint_cmd;

  state_publisher_->msg_.desired_joint_state = desired_joint_states;
  state_publisher_->msg_.actual_joint_state = current_joint_states;
  for (auto index = 0u; index < num_joints; ++index) {
    state_publisher_->msg_.error_joint_state.positions[index] = angles::shortest_angular_distance(
      current_joint_states.positions[index], desired_joint_states.positions[index]);
  }
  admittance_->get_controller_state(state_publisher_->msg_);
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  const auto num_joints = joint_names_.size();
  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (auto index = 0ul; index < num_joints; ++index) {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    // TODO(destogl): enable acceleration and remove next line
    state.accelerations.clear();
//     if (has_acceleration_state_interface_) {
//       assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
//     } else {
//       // Make empty so the property is ignored during interpolation
//       state.accelerations.clear();
//     }
  } else {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
}

bool AdmittanceController::read_state_from_command_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & output_state)
{
  bool has_values = true;
  const auto num_joints = joint_names_.size();
  trajectory_msgs::msg::JointTrajectoryPoint state = output_state;

  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
    {
      for (auto index = 0ul; index < num_joints; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface)
    {
      return std::find_if(
        joint_interface.begin(), joint_interface.end(),
        [](const auto & interface) {return std::isnan(interface.get().get_value());}) ==
             joint_interface.end();
    };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0])) {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  } else {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1])) {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
      //TODO(destogl): enable this line under to be sure if positions are not existing and velocities
      // are existing to still update the output_state; !commented because not tested!
//       has_values = true;
    } else {
      state.velocities.clear();
      has_values = false;
    }
  }
  else {
    state.velocities.clear();
  }

// TODO(destogl): Enable this
//   // Acceleration is used only in combination with velocity
//   if (has_acceleration_state_interface_) {
//     if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2])) {
//       assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
//     } else {
//       state.accelerations.clear();
//       has_values = false;
//     }
//   } else {
//     state.accelerations.clear();
//   }

  if (has_values) {
    output_state = state;
  }

  return has_values;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController,
  controller_interface::ControllerInterface)
