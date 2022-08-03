// Copyright (c) 2022, PickNik, Inc.
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
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#include "admittance_controller/admittance_controller.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include "admittance_controller/admittance_rule_impl.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace admittance_controller {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn AdmittanceController::on_init() {
    // initialize controller config
    try {
      admittance_ = std::make_unique<admittance_controller::AdmittanceRule>(get_node());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    num_joints_ = admittance_->parameters_.joints.size();

    // allocate dynamic memory
    last_state_reference_.positions.assign(num_joints_, 0.0);
    last_state_reference_.velocities.assign(num_joints_, 0.0);
    last_state_reference_.accelerations.assign(num_joints_, 0.0);
    last_commanded_state_ = last_state_reference_;
    state_reference_ = last_state_reference_;
    state_desired_ = last_state_reference_;
    state_current_ = last_state_reference_;

    joint_position_command_interface_.reserve(num_joints_);
    joint_velocity_command_interface_.reserve(num_joints_);
    joint_acceleration_command_interface_.reserve(num_joints_);
    joint_effort_command_interface_.reserve(num_joints_);
    joint_position_state_interface_.reserve(num_joints_);
    joint_velocity_state_interface_.reserve(num_joints_);
    joint_acceleration_state_interface_.reserve(num_joints_);
    joint_effort_state_interface_.reserve(num_joints_);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const {
    // Create an InterfaceConfiguration for the controller manager. The controller manager will then try to
    // claim the joint + interface combinations for state interfaces from the resource manager. Finally,
    // controller manager will populate the state_interfaces_ vector field via the ControllerInterfaceBase.
    // Note: state_interface_types_ contains position, velocity, acceleration; effort is not supported
    if (!admittance_) {
      return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    }

    std::vector<std::string> state_interfaces_config_names;
    for (const auto &interface: admittance_->parameters_.state_interfaces) {
      for (const auto &joint: admittance_->parameters_.joints) {
        state_interfaces_config_names.push_back(joint + "/" + interface);
      }
    }
    auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
    state_interfaces_config_names.insert(state_interfaces_config_names.end(), ft_interfaces.begin(),
                                         ft_interfaces.end());

    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            state_interfaces_config_names};
  }

  controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const {
    // Create an InterfaceConfiguration for the controller manager. The controller manager will then try to
    // claim the joint + interface combinations for command interfaces from the resource manager. Finally,
    // controller manager will populate the command_interfaces_ vector field via the ControllerInterfaceBase
    // Note: command_interface_types_ contains position, velocity; acceleration, effort are not supported
    if (!admittance_) {
      return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    }

    std::vector<std::string> command_interfaces_config_names;
    for (const auto &interface: admittance_->parameters_.command_interfaces) {
      for (const auto &joint: admittance_->parameters_.joints) {
        command_interfaces_config_names.push_back(joint + "/" + interface);
      }
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            command_interfaces_config_names};
  }

  std::vector<hardware_interface::CommandInterface> AdmittanceController::on_export_reference_interfaces() {
    // create CommandInterface interfaces that other controllers will be able to chain with
    if (!admittance_) {
      return {};
    }

    std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
    auto num_chainable_interfaces =
        admittance_->parameters_.chainable_command_interfaces.size() * admittance_->parameters_.joints.size();

    //allocate dynamic memory
    chainable_command_interfaces.reserve(num_chainable_interfaces);
    reference_interfaces_.resize(num_chainable_interfaces, std::numeric_limits<double>::quiet_NaN());
    position_reference_.reserve(num_joints_);
    velocity_reference_.reserve(num_joints_);

    // assign reference interfaces
    auto index = 0ul;
    for (const auto& interface : reference_interfaces_types_) {
      for (const auto& joint : admittance_->parameters_.joints) {
        if (hardware_interface::HW_IF_POSITION == interface)
          position_reference_.emplace_back(reference_interfaces_.data() + index);
        else if (hardware_interface::HW_IF_VELOCITY == interface) {
          velocity_reference_.emplace_back(reference_interfaces_.data() + index);
        }
        chainable_command_interfaces.emplace_back(
            hardware_interface::CommandInterface(std::string(get_node()->get_name()),
                                                 joint + "/" + interface,
                                                 reference_interfaces_.data() +
                                                 index++));
      }
    }

    return chainable_command_interfaces;;
  }

  CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    try {
      admittance_ = std::make_unique<admittance_controller::AdmittanceRule>(get_node());
      num_joints_ = admittance_->parameters_.joints.size();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    // print and validate interface types
    for (const auto &tmp: admittance_->parameters_.state_interfaces) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", ("state int types are: " + tmp + "\n").c_str());
    }
    for (const auto &tmp: admittance_->parameters_.command_interfaces) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", ("command int types are: " + tmp + "\n").c_str());
    }
    for (const auto &tmp: admittance_->parameters_.chainable_command_interfaces) {
      if (tmp == hardware_interface::HW_IF_POSITION || tmp == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(get_node()->get_logger(), "%s", ("chainable int types are: " + tmp + "\n").c_str());
      } else {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "chainable interface type %s is not supported. Supported types are %s and %s",
                     tmp.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }

    auto get_interface_list = [](const std::vector<std::string> &interface_types) {
      std::stringstream ss_command_interfaces;
      for (size_t index = 0; index < interface_types.size(); ++index) {
        if (index != 0) {
          ss_command_interfaces << " ";
        }
        ss_command_interfaces << interface_types[index];
      }
      return ss_command_interfaces.str();
    };
    RCLCPP_INFO(
        get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
        get_interface_list(admittance_->parameters_.command_interfaces).c_str(),
        get_interface_list(admittance_->parameters_.state_interfaces).c_str());

    // setup subscribers and publishers
    auto joint_command_callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg) {
      input_joint_command_.writeFromNonRT(msg);
    };
    input_joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "~/joint_commands", rclcpp::SystemDefaultsQoS(), joint_command_callback);
    s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
        "~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg >>(s_publisher_);

    // Initialize state message
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = admittance_->parameters_.joints;
    state_publisher_->msg_.actual_joint_state.positions.resize(num_joints_, 0.0);
    state_publisher_->msg_.desired_joint_state.positions.resize(num_joints_, 0.0);
    state_publisher_->msg_.error_joint_state.positions.resize(num_joints_, 0.0);
    state_publisher_->unlock();

    // Initialize FTS semantic semantic_component
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
        semantic_components::ForceTorqueSensor(admittance_->parameters_.ft_sensor.name));

    // configure admittance rule
    if (admittance_->configure(get_node(), num_joints_) == controller_interface::return_type::ERROR) {
      return CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::on_configure(previous_state);
  }

  CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    // on_activate is called when the lifecycle node activates.
    if (!admittance_) {
      return CallbackReturn::ERROR;
    }

    // update parameters if any have changed
    if (admittance_->parameter_handler_->is_old(admittance_->parameters_)) {
      admittance_->parameters_ = admittance_->parameter_handler_->get_params();
    }

    // reset all interfaces in case of restart
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_acceleration_command_interface_.clear();
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_acceleration_state_interface_.clear();
    joint_effort_state_interface_.clear();

    // assign command interfaces
    for (auto i = 0ul; i < admittance_->parameters_.command_interfaces.size(); i++) {
      for (auto j = 0ul; j < admittance_->parameters_.joints.size(); j++) {
        if (hardware_interface::HW_IF_POSITION == admittance_->parameters_.command_interfaces[i]) {
          joint_position_command_interface_.emplace_back(command_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_VELOCITY == admittance_->parameters_.command_interfaces[i]) {
          joint_velocity_command_interface_.emplace_back(command_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_ACCELERATION == admittance_->parameters_.command_interfaces[i]) {
          joint_acceleration_command_interface_.emplace_back(command_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_EFFORT == admittance_->parameters_.command_interfaces[i]) {
          joint_effort_command_interface_.emplace_back(command_interfaces_[i * num_joints_ + j]);
        }
      }
    }

    // assign state interfaces
    for (auto i = 0ul; i < admittance_->parameters_.state_interfaces.size(); i++) {
      for (auto j = 0ul; j < admittance_->parameters_.joints.size(); j++) {
        if (hardware_interface::HW_IF_POSITION == admittance_->parameters_.state_interfaces[i]) {
          joint_position_state_interface_.emplace_back(state_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_VELOCITY == admittance_->parameters_.state_interfaces[i]) {
          joint_velocity_state_interface_.emplace_back(state_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_ACCELERATION == admittance_->parameters_.state_interfaces[i]) {
          joint_acceleration_state_interface_.emplace_back(state_interfaces_[i * num_joints_ + j]);
        } else if (hardware_interface::HW_IF_EFFORT == admittance_->parameters_.state_interfaces[i]) {
          joint_effort_state_interface_.emplace_back(state_interfaces_[i * num_joints_ + j]);
        }
      }
    }

    // initialize interface of the FTS semantic semantic component
    force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

    // if joint_position_state_interface_ is empty, we have no information about the state, continuing is dangerous
    if (joint_position_state_interface_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No joint position state interface exist.");
      return CallbackReturn::ERROR;
    }

    // handle state after restart or initial startups
    read_state_from_hardware(state_current_);

    // initialize states from last read state reference
    last_commanded_state_ = state_current_;
    last_state_reference_ = state_current_;

    read_state_reference_interfaces(state_reference_);
    // reset dynamic fields in case non-zero
    state_reference_.velocities.assign(num_joints_, 0.0);
    state_reference_.accelerations.assign(num_joints_, 0.0);
    state_desired_ = state_reference_;

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type AdmittanceController::update_reference_from_subscribers() {
    // update input reference from ros subscriber message
    if (!admittance_) {
      return controller_interface::return_type::ERROR;
    }

    joint_command_msg = *input_joint_command_.readFromRT();

    // if message exists, load values into references
    if (joint_command_msg.get()) {
      for (auto i = 0ul; i < joint_command_msg->positions.size(); i++) {
        *position_reference_[i] = joint_command_msg->positions[i];
      }
      for (auto i = 0ul; i < joint_command_msg->velocities.size(); i++) {
        *velocity_reference_[i] = joint_command_msg->velocities[i];
      }
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type
  AdmittanceController::update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Realtime constraints are required in this function
    if (!admittance_) {
      return controller_interface::return_type::ERROR;
    }
    loop_counter++;

    // update input reference from chainable interfaces
    read_state_reference_interfaces(state_reference_);

    // get all controller inputs
    geometry_msgs::msg::Wrench ft_values;
    force_torque_sensor_->get_values_as_message(ft_values);
    read_state_from_hardware(state_current_);

    // apply admittance control to reference to determine desired state
    admittance_->update(state_current_, ft_values, state_reference_, period, state_desired_);

    // write calculated values to joint interfaces
    for (auto i = 0ul; i < joint_position_command_interface_.size(); i++) {
      joint_position_command_interface_[i].get().set_value(state_desired_.positions[i]);
      last_commanded_state_.positions[i] = state_desired_.positions[i];
    }
    for (auto i = 0ul; i < joint_velocity_command_interface_.size(); i++) {
      joint_velocity_command_interface_[i].get().set_value(state_desired_.velocities[i]);
      last_commanded_state_.velocities[i] = state_desired_.velocities[i];
    }
    for (auto i = 0ul; i < joint_acceleration_command_interface_.size(); i++) {
      joint_acceleration_command_interface_[i].get().set_value(state_desired_.accelerations[i]);
      last_commanded_state_.accelerations[i] = state_desired_.accelerations[i];
    }

    // Publish controller state
    state_publisher_->lock();
    state_publisher_->msg_.input_joint_command = pre_admittance_point;
    state_publisher_->msg_.desired_joint_state = state_desired_;
    state_publisher_->msg_.actual_joint_state = state_current_;
    state_publisher_->msg_.error_joint_state = state_error_;
    admittance_->get_controller_state(state_publisher_->msg_);
    state_publisher_->unlockAndPublish();

    return controller_interface::return_type::OK;
  }

  CallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    force_torque_sensor_->release_interfaces();

    return LifecycleNodeInterface::on_deactivate(previous_state);
  }

  CallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &previous_state) {
    if (!admittance_) {
      return CallbackReturn::ERROR;
    }
    admittance_->reset(num_joints_);
    return CallbackReturn::SUCCESS;
  }

  void AdmittanceController::read_state_from_hardware(
      trajectory_msgs::msg::JointTrajectoryPoint &state_reference) {
    // Fill fields of state_reference argument from hardware state interfaces. If the hardware does not exist or
    // the values are nan, the corresponding state field will be set to empty.

    // if any interface has nan values, assume state_reference is the last command state
    for (auto i = 0ul; i < joint_position_state_interface_.size(); i++) {
      state_reference.positions[i] = joint_position_state_interface_[i].get().get_value();
      if (std::isnan(state_reference.positions[i])) {
        state_reference.positions = last_commanded_state_.positions;
        break;
      }
    }
    for (auto i = 0ul; i < joint_velocity_state_interface_.size(); i++) {
      state_reference.velocities[i] = joint_velocity_state_interface_[i].get().get_value();
      if (std::isnan(state_reference.velocities[i])) {
        state_reference.velocities = last_commanded_state_.velocities;
        break;
      }
    }
    for (auto i = 0ul; i < joint_acceleration_state_interface_.size(); i++) {
      state_reference.accelerations[i] = joint_acceleration_state_interface_[i].get().get_value();
      if (std::isnan(state_reference.accelerations[i])) {
        state_reference.accelerations = last_commanded_state_.accelerations;
        break;
      }
    }

  }

  void AdmittanceController::read_state_reference_interfaces(
      trajectory_msgs::msg::JointTrajectoryPoint &state_reference) {
    // Fill fields of state_reference argument from controller references. If the interface does not exist or
    // the values are nan, the corresponding field will be set to empty

    for (auto i = 0ul; i < position_reference_.size(); i++) {
      if (std::isnan(*position_reference_[i])) {
        *position_reference_[i] = last_state_reference_.positions[i];
      }
      state_reference.positions[i] = *position_reference_[i];
    }
    last_state_reference_.positions = state_reference.positions;

    for (auto i = 0ul; i < velocity_reference_.size(); i++) {
      if (std::isnan(*velocity_reference_[i])) {
        *velocity_reference_[i] = last_state_reference_.velocities[i];
      }
      state_reference.velocities[i] = *velocity_reference_[i];
    }
    last_state_reference_.velocities = state_reference.velocities;

  }

  bool AdmittanceController::on_set_chained_mode(bool chained_mode) {
    // this method sets the chained mode to value of argument chained_mode if true is returned.
    return true;
  }

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(admittance_controller::AdmittanceController,
                       controller_interface::ChainableControllerInterface
)
