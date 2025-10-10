// Copyright (c) 2025, University of Salerno, Automatic Control Group
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
// Authors: Davide Risi

#include "gravity_compensation_pd_controller/gravity_compensation_pd_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

// URDF
#include <urdf/model.hpp>

namespace gravity_compensation_pd_controller
{
GravityCompensationPDController::GravityCompensationPDController()
: controller_interface::ChainableControllerInterface(),
  dynamics_solver_loader_(
    "inverse_dynamics_solver", "inverse_dynamics_solver::InverseDynamicsSolver"),
  dynamics_solver_(nullptr)
{
}

controller_interface::CallbackReturn GravityCompensationPDController::on_init()
{
  try
  {
    parameter_handler_ =
      std::make_shared<gravity_compensation_pd_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Number of joints to control is fixed after initialization
  num_joints_ = parameter_handler_->get_params().joints.size();

  // Retrieve torque limits from URDF
  if (!read_joint_effort_limits_from_urdf())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint effort limits from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Allocate dynamic memory
  robot_joint_state_.positions.assign(num_joints_, 0.0);
  robot_joint_state_.velocities.assign(num_joints_, 0.0);
  joint_command_.assign(num_joints_, 0.0);
  joint_reference_.assign(num_joints_, 0.0);
  last_joint_reference_ = joint_reference_;

  // Allocate Eigen vectors' dynamic memory for real-time safeness
  position_error_.resize(num_joints_);
  eigen_effort_command_.resize(num_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationPDController::command_interface_configuration() const
{
  const auto params = parameter_handler_->get_params();
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params.joints.size());

  if (params.command_interfaces_names_override.effort.size() == num_joints_)
  {
    command_interfaces_config.names.insert(
      command_interfaces_config.names.end(),
      params.command_interfaces_names_override.effort.begin(),
      params.command_interfaces_names_override.effort.end());
  }
  else
  {
    for (const auto & joint_name : params.joints)
    {
      command_interfaces_config.names.push_back(joint_name + "/effort");
    }
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
GravityCompensationPDController::state_interface_configuration() const
{
  const auto params = parameter_handler_->get_params();
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params.joints.size() * 2);

  if (params.state_interfaces_names_override.position.size() == num_joints_)
  {
    state_interfaces_config.names.insert(
      state_interfaces_config.names.end(), params.state_interfaces_names_override.position.begin(),
      params.state_interfaces_names_override.position.end());
  }
  else
  {
    for (const auto & joint_name : params.joints)
    {
      state_interfaces_config.names.push_back(joint_name + "/position");
    }
  }

  if (params.state_interfaces_names_override.velocity.size() == num_joints_)
  {
    state_interfaces_config.names.insert(
      state_interfaces_config.names.end(), params.state_interfaces_names_override.velocity.begin(),
      params.state_interfaces_names_override.velocity.end());
  }
  else
  {
    for (const auto & joint_name : params.joints)
    {
      state_interfaces_config.names.push_back(joint_name + "/velocity");
    }
  }
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
GravityCompensationPDController::on_export_reference_interfaces()
{
  const auto params = parameter_handler_->get_params();
  ChainableControllerInterface::reference_interfaces_.assign(
    num_joints_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(num_joints_);

  for (std::size_t i = 0; i < params.joints.size(); ++i)
  {
    reference_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        std::string(get_node()->get_name()) + "/" + params.joints[i], "position",
        &ChainableControllerInterface::reference_interfaces_[i]));
  }
  return reference_interfaces;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set the command to NaN values, to notify the hardware that the controller is unable to provide
  // valid commands
  std::fill(joint_command_.begin(), joint_command_.end(), std::numeric_limits<double>::quiet_NaN());
  if (!write_command_interfaces_())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to write NaN values to command interfaces while in error state.");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_ERROR(
    get_node()->get_logger(),
    "Controller is in error state. Writing NaN values to command interfaces. Restart the "
    "controller to recover.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  gravity_compensation_pd_controller::Params params = parameter_handler_->get_params();

  if (dynamics_solver_ == nullptr)
  {
    try
    {
      dynamics_solver_ =
        dynamics_solver_loader_.createSharedInstance(params.dynamics_solver.dynamics_solver_plugin);
    }
    catch (pluginlib::PluginlibException & ex)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Exception while loading the dynamics solver plugin '%s': '%s'",
        params.dynamics_solver.dynamics_solver_plugin.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Set the robot description as node parameters
  get_node()->declare_parameter("robot_description", get_robot_description());

  // Initialize the dynamics solver
  try
  {
    dynamics_solver_->initialize(get_node()->get_node_parameters_interface(), "dynamics_solver");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception while initializing the dynamics solver plugin '%s': '%s'",
      params.dynamics_solver.dynamics_solver_plugin.c_str(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params.p_gains.size() != num_joints_ || params.d_gains.size() != num_joints_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Gains size does not match joints size");
    return controller_interface::CallbackReturn::ERROR;
  }

  Kp_.diagonal() = Eigen::Map<const Eigen::VectorXd>(params.p_gains.data(), params.p_gains.size());
  Kd_.diagonal() = Eigen::Map<const Eigen::VectorXd>(params.d_gains.data(), params.d_gains.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read the current state of the robot from the hardware
  if (!read_state_interfaces_())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Use current joint state as first valid reference
  joint_reference_ = robot_joint_state_.positions;
  last_joint_reference_ = joint_reference_;
  std::fill(joint_command_.begin(), joint_command_.end(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GravityCompensationPDController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type GravityCompensationPDController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read the current state of the robot from the hardware
  if (!read_state_interfaces_())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint states from the hardware.\n");
    return controller_interface::return_type::ERROR;
  }

  // Read the reference from the reference interfaces
  if (!read_reference_interfaces_())
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_,
      "Received NaN values in joint reference positions. Using last valid reference instead.");
    // If NaN values are detected in the joint reference positions, use the last valid reference
    joint_reference_ = last_joint_reference_;
  }

  compute_control_law_();

  // Write the command to the hardware
  if (!write_command_interfaces_())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to write joint command to the hardware.\n");
    return controller_interface::return_type::ERROR;
  }

  // Update the last command and reference
  last_joint_reference_ = joint_reference_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityCompensationPDController::on_set_chained_mode(bool /* chained_mode */) { return true; }

void GravityCompensationPDController::compute_control_law_()
{
  Eigen::VectorXd current_positions = Eigen::Map<const Eigen::VectorXd>(
    robot_joint_state_.positions.data(), robot_joint_state_.positions.size());
  Eigen::VectorXd current_velocities = Eigen::Map<const Eigen::VectorXd>(
    robot_joint_state_.velocities.data(), robot_joint_state_.velocities.size());

  for (std::size_t i = 0; i < num_joints_; i++)
  {
    position_error_(i) = joint_reference_[i] - current_positions(i);
  }

  // Compute the effort command using PD control law
  if (parameter_handler_->get_params().compensate_gravity)
  {
    eigen_effort_command_ = dynamics_solver_->getGravityVector(current_positions);
  }

  eigen_effort_command_ += Kp_ * position_error_ - Kd_ * current_velocities;

  // Apply torque limits using Eigen
  eigen_effort_command_ = eigen_effort_command_.cwiseMax(-torque_limits_).cwiseMin(torque_limits_);
  std::copy(
    eigen_effort_command_.data(), eigen_effort_command_.data() + num_joints_,
    joint_command_.begin());
}

bool GravityCompensationPDController::read_state_interfaces_()
{
  bool all_read{true};
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    std::optional<double> state_position_op =
      ControllerInterfaceBase::state_interfaces_[i].get_optional();
    std::optional<double> state_velocity_op =
      ControllerInterfaceBase::state_interfaces_[i + num_joints_].get_optional();

    if (!state_position_op.has_value() || !state_velocity_op.has_value())
    {
      all_read = false;
    }
    robot_joint_state_.positions[i] = state_position_op.value();
    robot_joint_state_.velocities[i] = state_velocity_op.value();
  }
  return all_read;
}

bool GravityCompensationPDController::write_command_interfaces_()
{
  bool all_written{true};
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    all_written &= ControllerInterfaceBase::command_interfaces_[i].set_value(joint_command_[i]);
  }
  return all_written;
}

bool GravityCompensationPDController::read_reference_interfaces_()
{
  bool all_read{true};
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    if (std::isnan(ChainableControllerInterface::reference_interfaces_[i]))
    {
      all_read = false;
    }
    joint_reference_[i] = ChainableControllerInterface::reference_interfaces_[i];
  }
  return all_read;
}

bool GravityCompensationPDController::read_joint_effort_limits_from_urdf()
{
  torque_limits_.resize(num_joints_);
  const std::string & robot_description = get_robot_description();
  const std::vector<std::string> & joint_names = parameter_handler_->get_params().joints;

  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Robot description is empty.");
    return false;
  }

  urdf::Model robot_model;
  if (!robot_model.initString(robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF.");
    return false;
  }

  std::size_t num_joints{joint_names.size()};
  std::vector<std::string> joints_without_limits;

  for (std::size_t i = 0; i < num_joints; ++i)
  {
    const std::string & joint_name{joint_names[i]};
    urdf::JointConstSharedPtr joint{robot_model.getJoint(joint_name)};

    if (!joint) return false;

    if (joint->limits)
    {
      torque_limits_[i] = joint->limits->effort;
    }
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Joint '%s' does not have effort limits specified in the URDF. Setting to infinity.",
        joint_name.c_str());
      torque_limits_[i] = std::numeric_limits<double>::max();
    }
  }

  return true;
}

}  // namespace gravity_compensation_pd_controller

PLUGINLIB_EXPORT_CLASS(
  gravity_compensation_pd_controller::GravityCompensationPDController,
  controller_interface::ChainableControllerInterface)
