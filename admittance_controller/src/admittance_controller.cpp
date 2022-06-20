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

#include <math.h>
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

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


  AdmittanceController::AdmittanceController() = default;

  void AdmittanceController::wrench_stamped_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {
    if (controller_is_active_) {
      rtBuffers.input_wrench_command_.writeFromNonRT(msg);
    }
  }

  void
  AdmittanceController::joint_command_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg) {
    if (controller_is_active_) {
      rtBuffers.input_joint_command_.writeFromNonRT(msg);
    }
  }

  CallbackReturn AdmittanceController::on_init() {
//    // set chained mode
//    set_chained_mode(true);
    // initialize RTbuffers
    rtBuffers.input_joint_command_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>());
    rtBuffers.input_wrench_command_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::WrenchStamped>());
    // initialize controller parameters
    admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();
    admittance_->parameters_.initialize(get_node());
    // TODO need error handling
    params = {
        get_node()->get_parameter("joints").as_string_array(),
        get_node()->get_parameter("command_interfaces").as_string_array(),
        get_node()->get_parameter("state_interfaces").as_string_array(),
        get_node()->get_parameter("chainable_command_interfaces").as_string_array(),
        get_node()->get_parameter("ft_sensor_name").as_string(),
        get_node()->get_parameter("use_joint_commands_as_input").as_bool(),
        get_node()->get_parameter("joint_limiter_type").as_string(),
        get_node()->get_parameter("allow_partial_joints_goal").as_bool(),
        get_node()->get_parameter("allow_integration_in_goal_trajectories").as_bool(),
        get_node()->get_parameter("action_monitor_rate").as_double(),
        get_node()->get_parameter("open_loop_control").as_bool(),
    };

    for (const auto &tmp: params.state_interface_types_) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", ("state int types are: " + tmp + "\n").c_str());
    }
    for (const auto &tmp: params.command_interface_types_) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", ("command int types are: " + tmp + "\n").c_str());
    }
    for (const auto &tmp: params.chainable_command_interface_types_) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", ("chainable int types are: " + tmp + "\n").c_str());
    }

    try {
      admittance_->parameters_.declare_parameters();
    } catch (const std::exception &e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    num_joints_ = params.joint_names_.size();


    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const {
    // Create an InterfaceConfiguration for the controller manager. The controller manager will then try to
    // claim the joint + interface combinations for state interfaces from the resource manager. Finally,
    // controller manager will populate the state_interfaces_ vector field via the ControllerInterfaceBase.
    // Note: state_interface_types_ contains position, velocity, acceleration; effort is not supported

    std::vector<std::string> state_interfaces_config_names; //= force_torque_sensor_->get_state_interface_names();

    for (const auto &interface: params.state_interface_types_) {
      for (const auto &joint: params.joint_names_) {
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

    std::vector<std::string> command_interfaces_config_names;//(joint_names_.size() * command_interface_types_.size());

    for (const auto &interface: params.command_interface_types_) {
      for (const auto &joint: params.joint_names_) {
        command_interfaces_config_names.push_back(joint + "/" + interface);
      }
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            command_interfaces_config_names};
  }

  std::vector<hardware_interface::CommandInterface> AdmittanceController::on_export_reference_interfaces() {

    std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
    auto num_chainable_interfaces = params.chainable_command_interface_types_.size() * params.joint_names_.size();
    reference_interfaces_.resize(num_chainable_interfaces, std::numeric_limits<double>::quiet_NaN());
    chainable_command_interfaces.reserve(num_chainable_interfaces);

    auto index = 0ul;
    for (const auto &interface: params.chainable_command_interface_types_) {
      for (const auto &joint: params.joint_names_) {
        chainable_command_interfaces.emplace_back(hardware_interface::CommandInterface(std::string(get_node()->get_name()),
                                                                         joint + "/" + interface,
                                                                                       reference_interfaces_.data() +
                                                                                       index++));
      }
    }

    joint_position_chainable_interface_.clear();
    joint_velocity_chainable_interface_.clear();
    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::CommandInterface>> *> chainable_interface_map = {
        {hardware_interface::HW_IF_POSITION, &joint_position_chainable_interface_},
        {hardware_interface::HW_IF_VELOCITY, &joint_velocity_chainable_interface_}
    };

    for (auto i = 0ul; i < params.chainable_command_interface_types_.size(); i++) {
      for (auto j = 0ul; j < params.joint_names_.size(); j++) {
        chainable_interface_map[params.chainable_command_interface_types_[i]]->emplace_back(
            chainable_command_interfaces[i * num_joints_ + j]);
      }
    }

    return chainable_command_interfaces;
  }

  controller_interface::return_type AdmittanceController::update_reference_from_subscribers() {
    // update input reference from ros subscriber message
    return controller_interface::return_type::OK; //TODO fix this

    joint_command_msg = *rtBuffers.input_joint_command_.readFromRT();
    for (auto i = 0ul; i < joint_command_msg->positions.size(); i++) {
      joint_position_chainable_interface_[i].get().set_value(joint_command_msg->positions[i]);
    }
    for (auto i = 0ul; i < joint_command_msg->velocities.size(); i++) {
      joint_velocity_chainable_interface_[i].get().set_value(joint_command_msg->velocities[i]);
    }
    for (auto i = 0ul; i < joint_command_msg->accelerations.size(); i++) {
      joint_acceleration_chainable_interface_[i].get().set_value(joint_command_msg->accelerations[i]);
    }

    return controller_interface::return_type::OK;
  }

  bool AdmittanceController::on_set_chained_mode(bool chained_mode) {
    // this method set chained mode to value of chained_mode if true is returned
    return true;
  }

  CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    // load and set all ROS parameters
    if (!admittance_->parameters_.get_parameters()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Error happened during reading parameters");
      return CallbackReturn::ERROR;
    }

    // Print output so users can be sure the interface setup is correct
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
        get_interface_list(params.command_interface_types_).c_str(),
        get_interface_list(params.state_interface_types_).c_str());

    RCLCPP_INFO(get_node()->get_logger(), "Action status changes will be monitored at %.2f Hz.",
                params.action_monitor_rate_);
    if (params.use_joint_commands_as_input_) {
      RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Admittance controller does not support non-joint input modes.");
      return CallbackReturn::ERROR;
    }

    // setup subscribers and publishers
    input_wrench_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "~/ft_data", rclcpp::SystemDefaultsQoS(),
        std::bind(&AdmittanceController::wrench_stamped_callback, this, std::placeholders::_1));
    input_joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "~/pose_commands", rclcpp::SystemDefaultsQoS(),
        std::bind(&AdmittanceController::joint_command_callback, this, std::placeholders::_1));
    s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
        "~/state", rclcpp::SystemDefaultsQoS());
    rtBuffers.state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);

    // Initialize state message
    rtBuffers.state_publisher_->lock();
    rtBuffers.state_publisher_->msg_.joint_names = params.joint_names_;
    rtBuffers.state_publisher_->msg_.actual_joint_state.positions.resize(num_joints_, 0.0);
    rtBuffers.state_publisher_->msg_.desired_joint_state.positions.resize(num_joints_, 0.0);
    rtBuffers.state_publisher_->msg_.error_joint_state.positions.resize(num_joints_, 0.0);
    rtBuffers.state_publisher_->unlock();
    // Initialize FTS semantic semantic_component
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
        semantic_components::ForceTorqueSensor(params.ft_sensor_name_));

    // configure admittance rule
    admittance_->configure(get_node(), num_joints_);
    // HACK: This is workaround because it seems that updating parameters only in `on_activate` does
    // not work properly: why?
    admittance_->parameters_.update();

    return LifecycleNodeInterface::on_configure(previous_state);
  }


  CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    // on_activate is called when the lifecycle activates. Realtime constraints are required.
    controller_is_active_ = true;
    joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
    joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
    // assign state interfaces
    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *> command_interface_map = {
        {hardware_interface::HW_IF_POSITION, &joint_position_command_interface_},
        {hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_interface_}
    };

    for (auto i = 0ul; i < params.command_interface_types_.size(); i++) {
      for (auto j = 0ul; j < params.joint_names_.size(); j++) {
        command_interface_map[params.command_interface_types_[i]]->emplace_back(
            command_interfaces_[i * num_joints_ + j]);
      }
    }

    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *> state_interface_map = {
        {hardware_interface::HW_IF_POSITION, &joint_position_state_interface_},
        {hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_interface_}
    };

    for (auto i = 0ul; i < params.state_interface_types_.size(); i++) {
      for (auto j = 0ul; j < params.joint_names_.size(); j++) {
        state_interface_map[params.state_interface_types_[i]]->emplace_back(state_interfaces_[i * num_joints_ + j]);
      }
    }

    // Initialize interface of the FTS semantic semantic component
    force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
    // Initialize Admittance Rule from current states
    admittance_->reset();

    // if in open loop mode, the position state interface should be ignored
    if (params.open_loop_control_) {
//        state_offset_.positions.resize(joint_position_state_interface_.size(), 0.0);
//        for (int i=0; i < num_joints_; i++){
//          state_offset_.positions[i] = joint_position_state_interface_[i].get().get_value();
//        }
//        open_loop_buffer.resize(joint_position_state_interface_.size(), 0.0);

      open_loop_buffer.resize(joint_position_state_interface_.size(), 0.0);
      for (int i = 0; i < num_joints_; i++) {
        open_loop_buffer[i] = joint_position_state_interface_[i].get().get_value();
      }
    }

    // Handle state after restart or initial startup
    read_state_from_hardware(last_state_reference_);
    // if last_state_reference_ is empty, we have no information about the state, assume zero
    if (last_state_reference_.positions.empty()) last_state_reference_.positions.resize(num_joints_, 0.0);
    read_state_from_command_interfaces(last_commanded_state_);
    // if last_commanded_state_ is empty, then our safest option is to set it to the last reference
    if (last_commanded_state_.positions.empty()) last_commanded_state_.positions = last_state_reference_.positions;

    // reset dynamic fields in case non-zero
    if (last_state_reference_.velocities.empty()) last_state_reference_.velocities.assign(num_joints_, 0.0);
    if (last_state_reference_.accelerations.empty()) last_state_reference_.accelerations.assign(num_joints_, 0.0);
    if (last_commanded_state_.velocities.empty()) last_commanded_state_.velocities.assign(num_joints_, 0.0);
    if (last_commanded_state_.accelerations.empty()) last_commanded_state_.accelerations.assign(num_joints_, 0.0);
    // initialize state reference
    state_reference_ = last_state_reference_;

    // if there are no state position interfaces, then force use of open loop control
    if (joint_position_state_interface_.empty() && !params.open_loop_control_) {
      params.open_loop_control_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
                  "control loop control set to true because no position state interface was provided.");
    }

    return CallbackReturn::SUCCESS;;
  }

  controller_interface::return_type
  AdmittanceController::update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Realtime constraints are required in this function

    // update input reference from chainable interfaces
    read_state_reference_from_chainable_interfaces(state_reference_);

    // sense: get all controller inputs
    geometry_msgs::msg::Wrench ft_values;
    force_torque_sensor_->get_values_as_message(ft_values);
    read_state_from_hardware(state_current_);

    // if values are not available, assume that the current state is the last commanded
    if (state_current_.positions.empty()) { state_current_.positions = last_commanded_state_.positions; }
    if (state_current_.velocities.empty()) { state_current_.velocities = last_commanded_state_.velocities; }
    if (state_current_.accelerations.empty()) { state_current_.accelerations = last_commanded_state_.accelerations; }


    // command: determine desired state from trajectory or pose goal
    // and apply admittance controller

    admittance_->update(state_current_, ft_values, state_reference_, period, state_desired_);

    // write calculated values to joint interfaces
    // at goal time (end of trajectory), check goal reference error and send fail to
    // action server out of tolerance
    for (auto i = 0ul; i < joint_position_command_interface_.size(); i++) {
      joint_position_command_interface_[i].get().set_value(state_desired_.positions[i]);
      if(params.open_loop_control_){
        open_loop_buffer[i] = state_desired_.positions[i];
      }
      last_commanded_state_.positions[i] = state_desired_.positions[i];
    }
    for (auto i = 0ul; i < joint_velocity_command_interface_.size(); i++) {
      double dt = 1.0 / 60; // hack!
      joint_velocity_command_interface_[i].get().set_value(state_desired_.velocities[i]);
      last_commanded_state_.velocities[i] = state_desired_.velocities[i];
      last_commanded_state_.positions[i] = state_current_.positions[i] + state_desired_.velocities[i] * dt; // hack!
      if(params.open_loop_control_){
        open_loop_buffer[i] = state_current_.positions[i] + state_desired_.velocities[i] * dt; // hack!
      }
    }
    for (auto i = 0ul; i < joint_acceleration_command_interface_.size(); i++) {
      joint_acceleration_command_interface_[i].get().set_value(state_desired_.accelerations[i]);
      last_commanded_state_.accelerations[i] = state_desired_.accelerations[i];
    }

    // save state reference before applying admittance rule TODO why is this here?
    //pre_admittance_point.points[0] = state_reference_; TODO fix this
    // Publish controller state
    rtBuffers.state_publisher_->lock();
    rtBuffers.state_publisher_->msg_.input_joint_command = pre_admittance_point;
    rtBuffers.state_publisher_->msg_.desired_joint_state = state_desired_;
    rtBuffers.state_publisher_->msg_.actual_joint_state = state_current_;
    rtBuffers.state_publisher_->msg_.error_joint_state = state_error_;
    admittance_->get_controller_state(rtBuffers.state_publisher_->msg_);
    rtBuffers.state_publisher_->unlockAndPublish();

    return controller_interface::return_type::OK;
  }

  CallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    controller_is_active_ = false;
    force_torque_sensor_->release_interfaces();

    return LifecycleNodeInterface::on_deactivate(previous_state);
  }

  CallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &previous_state) {
    admittance_->reset();
    return CallbackReturn::SUCCESS;
  }

  void AdmittanceController::read_state_from_hardware(
      trajectory_msgs::msg::JointTrajectoryPoint &state_current) {
    // Fill fields of state argument from hardware state interfaces. If the hardware does not exist,
    // the values are nan, that corresponding state field will be set to empty. If running in open loop
    // control, all states fields will be empty

    state_current.positions.resize(joint_position_state_interface_.size());
    state_current.velocities.resize(joint_velocity_state_interface_.size());
    state_current.accelerations.resize(joint_acceleration_state_interface_.size());

    // fill state message with values from hardware state interfaces
    for (auto i = 0ul; i < joint_position_state_interface_.size(); i++) {
      if (params.open_loop_control_) {
        state_current.positions[i] = open_loop_buffer[i];
      } else {
        state_current.positions[i] = joint_position_state_interface_[i].get().get_value();
      }
      if (std::isnan(state_current.positions[i])) {
        state_current.positions.clear();
        break;
      }
    }
    for (auto i = 0ul; i < joint_velocity_state_interface_.size(); i++) {
      state_current.velocities[i] = joint_velocity_state_interface_[i].get().get_value();
      if (std::isnan(state_current.velocities[i])) {
        state_current.velocities.clear();
        break;
      }
    }
    for (auto i = 0ul; i < joint_acceleration_state_interface_.size(); i++) {
      state_current.accelerations[i] = joint_acceleration_state_interface_[i].get().get_value();
      if (std::isnan(state_current.accelerations[i])) {
        state_current.accelerations.clear();
        break;
      }
    }
  }

  void AdmittanceController::read_state_from_command_interfaces(
      trajectory_msgs::msg::JointTrajectoryPoint &commanded_state) {
    // Fill fields of state argument from hardware command interfaces. If the interface does not exist or
    // the values are nan, that corresponding state field will be set to empty

    commanded_state.positions.resize(joint_position_command_interface_.size(), 0.0);
    commanded_state.velocities.resize(joint_velocity_command_interface_.size(), 0.0);
    commanded_state.accelerations.resize(joint_acceleration_command_interface_.size(), 0.0);

    // fill state message with values from hardware command interfaces
    for (auto i = 0ul; i < joint_position_command_interface_.size(); i++) {
      commanded_state.positions[i] = joint_position_command_interface_[i].get().get_value();
      if (std::isnan(commanded_state.positions[i])) {
        commanded_state.positions.clear();
        break;
      }
    }
    for (auto i = 0ul; i < joint_velocity_command_interface_.size(); i++) {
      commanded_state.velocities[i] = joint_velocity_command_interface_[i].get().get_value();
      if (std::isnan(commanded_state.velocities[i])) {
        commanded_state.velocities.clear();
        break;
      }
    }
    for (auto i = 0ul; i < joint_acceleration_command_interface_.size(); i++) {
      commanded_state.accelerations[i] = joint_acceleration_command_interface_[i].get().get_value();
      if (std::isnan(commanded_state.accelerations[i])) {
        commanded_state.accelerations.clear();
        break;
      }
    }
  }

  void AdmittanceController::read_state_reference_from_chainable_interfaces(
      trajectory_msgs::msg::JointTrajectoryPoint &state_reference) {
    // Fill fields of state argument from hardware command interfaces. If the interface does not exist or
    // the values are nan, that corresponding state field will be set to empty

    state_reference.positions.resize(joint_position_chainable_interface_.size(), 0.0);
    state_reference.velocities.resize(joint_velocity_chainable_interface_.size(), 0.0);
    state_reference.accelerations.resize(joint_acceleration_chainable_interface_.size(), 0.0);

    for (auto i = 0ul; i < joint_position_chainable_interface_.size(); i++) {
      state_reference.positions[i] = joint_position_chainable_interface_[i].get().get_value();
      if (std::isnan(state_reference.positions[i])) {
        state_reference.positions = last_state_reference_.positions;
        break;
      }
    }
    for (auto i = 0ul; i < joint_velocity_chainable_interface_.size(); i++) {
      state_reference.velocities[i] = joint_velocity_chainable_interface_[i].get().get_value();
      if (std::isnan(state_reference.velocities[i])) {
        state_reference.velocities = last_state_reference_.velocities;
        break;
      }
    }
    for (auto i = 0ul; i < joint_acceleration_chainable_interface_.size(); i++) {
      state_reference.accelerations[i] = joint_acceleration_chainable_interface_[i].get().get_value();
      if (std::isnan(state_reference.accelerations[i])) {
        state_reference.accelerations = last_state_reference_.accelerations;
        break;
      }
    }

  }

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(admittance_controller::AdmittanceController,
                       controller_interface::ChainableControllerInterface)
