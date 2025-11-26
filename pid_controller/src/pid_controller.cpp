// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Daniel Azanov, Dr. Denis
//

#include "pid_controller/pid_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_msgs/msg/single_dof_state.hpp"
#include "rclcpp/version.h"

namespace
{  // utility

// Changed services history QoS to keep all so we don't lose any client service calls
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 17
rclcpp::QoS qos_services =
  rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
    .reliable()
    .durability_volatile();
#else
static const rmw_qos_profile_t qos_services = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};
#endif

using ControllerCommandMsg = pid_controller::PidController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  ControllerCommandMsg & msg, const std::vector<std::string> & dof_names)
{
  msg.dof_names = dof_names;
  msg.values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg.values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg(
  ControllerCommandMsg & msg, const std::vector<std::string> & dof_names)
{
  reset_controller_reference_msg(msg, dof_names);
}

}  // namespace

namespace pid_controller
{
PidController::PidController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<pid_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::configure_parameters()
{
  params_ = param_listener_->get_params();

  if (!params_.reference_and_state_dof_names.empty())
  {
    reference_and_state_dof_names_ = params_.reference_and_state_dof_names;
  }
  else
  {
    reference_and_state_dof_names_ = params_.dof_names;
  }

  if (params_.dof_names.size() != reference_and_state_dof_names_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'dof_names' (%zu) and 'reference_and_state_dof_names' (%zu) parameters has to be "
      "the same!",
      params_.dof_names.size(), reference_and_state_dof_names_.size());
    return CallbackReturn::FAILURE;
  }

  dof_ = params_.dof_names.size();

  // TODO(destogl): is this even possible? Test it...
  if (params_.gains.dof_names_map.size() != dof_)
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'gains' (%zu) map and number or 'dof_names' (%zu) have to be the same!",
      params_.gains.dof_names_map.size(), dof_);
    return CallbackReturn::FAILURE;
  }

  pids_.resize(dof_);

  for (size_t i = 0; i < dof_; ++i)
  {
    pids_[i] = std::make_shared<control_toolbox::PidROS>(
      get_node(), "gains." + params_.dof_names[i], "~/" + params_.dof_names[i], false);
    if (!pids_[i]->initialize_from_ros_parameters())
    {
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reference_and_state_dof_names_.clear();
  pids_.clear();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&PidController::reference_callback, this, std::placeholders::_1));

  reset_controller_reference_msg(current_ref_, reference_and_state_dof_names_);
  input_ref_.set(current_ref_);

  // input state Subscriber and callback
  if (params_.use_external_measured_states)
  {
    auto measured_state_callback =
      [&](const std::shared_ptr<ControllerMeasuredStateMsg> state_msg) -> void
    {
      if (state_msg->dof_names.size() != reference_and_state_dof_names_.size())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Size of input data names (%zu) is not matching the expected size (%zu).",
          state_msg->dof_names.size(), reference_and_state_dof_names_.size());
        return;
      }
      if (state_msg->values.size() != reference_and_state_dof_names_.size())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Size of input data values (%zu) is not matching the expected size (%zu).",
          state_msg->values.size(), reference_and_state_dof_names_.size());
        return;
      }

      if (!state_msg->values_dot.empty())
      {
        if (params_.reference_and_state_interfaces.size() != 2)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "The reference_and_state_interfaces parameter has to have two interfaces [the "
            "interface and the derivative of the interface], in order to use the values_dot "
            "field.");
          return;
        }
        if (state_msg->values_dot.size() != reference_and_state_dof_names_.size())
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Size of input data values_dot (%zu) is not matching the expected size (%zu).",
            state_msg->values_dot.size(), reference_and_state_dof_names_.size());
          return;
        }
      }
      // TODO(destogl): Sort the input values based on joint and interface names
      measured_state_.set(*state_msg);
    };
    measured_state_subscriber_ = get_node()->create_subscription<ControllerMeasuredStateMsg>(
      "~/measured_state", subscribers_qos, measured_state_callback);
  }

  ControllerMeasuredStateMsg measured_state_msg;
  reset_controller_measured_state_msg(measured_state_msg, reference_and_state_dof_names_);
  measured_state_.set(measured_state_msg);

  measured_state_values_.resize(
    dof_ * params_.reference_and_state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher
  state_msg_.dof_states.resize(reference_and_state_dof_names_.size());
  for (size_t i = 0; i < reference_and_state_dof_names_.size(); ++i)
  {
    state_msg_.dof_states[i].name = reference_and_state_dof_names_[i];
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void PidController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->dof_names.empty() && msg->values.size() == reference_and_state_dof_names_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Reference massage does not have DoF names defined. "
      "Assuming that value have order as defined state DoFs");
    auto ref_msg = msg;
    ref_msg->dof_names = reference_and_state_dof_names_;
    input_ref_.set(*ref_msg);
  }
  else if (
    msg->dof_names.size() == reference_and_state_dof_names_.size() &&
    msg->values.size() == reference_and_state_dof_names_.size())
  {
    auto ref_msg = msg;  // simple initialization

    // sort values in the ref_msg
    reset_controller_reference_msg(*msg, reference_and_state_dof_names_);

    bool all_found = true;
    for (size_t i = 0; i < msg->dof_names.size(); ++i)
    {
      auto found_it =
        std::find(ref_msg->dof_names.begin(), ref_msg->dof_names.end(), msg->dof_names[i]);
      if (found_it == msg->dof_names.end())
      {
        all_found = false;
        RCLCPP_WARN(
          get_node()->get_logger(), "DoF name '%s' not found in the defined list of state DoFs.",
          msg->dof_names[i].c_str());
        break;
      }

      auto position = static_cast<size_t>(std::distance(ref_msg->dof_names.begin(), found_it));
      ref_msg->values[position] = msg->values[i];
      ref_msg->values_dot[position] = msg->values_dot[i];
    }

    if (all_found)
    {
      input_ref_.set(*ref_msg);
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of input data names (%zu) and/or values (%zu) is not matching the expected size (%zu).",
      msg->dof_names.size(), msg->values.size(), reference_and_state_dof_names_.size());
  }
}

controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.dof_names.size());
  for (const auto & dof_name : params_.dof_names)
  {
    command_interfaces_config.names.push_back(dof_name + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (params_.use_external_measured_states)
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(dof_ * params_.reference_and_state_interfaces.size());
    for (const auto & interface : params_.reference_and_state_interfaces)
    {
      for (const auto & dof_name : reference_and_state_dof_names_)
      {
        state_interfaces_config.names.push_back(dof_name + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> PidController::on_export_reference_interfaces()
{
  const size_t dof_reference_size = dof_ * params_.reference_and_state_interfaces.size();

  size_t total_reference_size = dof_reference_size;
  if (params_.export_gain_references)
  {
    total_reference_size += dof_ * GAIN_INTERFACES.size();
  }
  reference_interfaces_.resize(total_reference_size, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(total_reference_size);

  size_t index = 0;
  for (const auto & interface : params_.reference_and_state_interfaces)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      reference_interfaces.push_back(
        hardware_interface::CommandInterface(
          std::string(get_node()->get_name()) + "/" + dof_name, interface,
          &reference_interfaces_[index]));
      ++index;
    }
  }

  if (params_.export_gain_references)
  {
    size_t gains_start_index = dof_reference_size;
    for (const auto & gain_name : GAIN_INTERFACES)
    {
      for (const auto & dof_name : reference_and_state_dof_names_)
      {
        reference_interfaces.push_back(
          hardware_interface::CommandInterface(
            std::string(get_node()->get_name()) + "/" + dof_name, gain_name,
            &reference_interfaces_[gains_start_index]));
        ++gains_start_index;
      }
    }
  }

  return reference_interfaces;
}

std::vector<hardware_interface::StateInterface> PidController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(state_interfaces_values_.size());

  state_interfaces_values_.resize(
    reference_and_state_dof_names_.size() * params_.reference_and_state_interfaces.size(),
    std::numeric_limits<double>::quiet_NaN());
  size_t index = 0;
  for (const auto & interface : params_.reference_and_state_interfaces)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      state_interfaces.push_back(
        hardware_interface::StateInterface(
          std::string(get_node()->get_name()) + "/" + dof_name, interface,
          &state_interfaces_values_[index]));
      ++index;
    }
  }
  return state_interfaces;
}

bool PidController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::CallbackReturn PidController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  auto input_ref_op = input_ref_.try_get();
  if (input_ref_op.has_value())
  {
    current_ref_ = input_ref_op.value();
    reset_controller_reference_msg(current_ref_, reference_and_state_dof_names_);
    input_ref_.try_set(current_ref_);
  }
  auto measured_state_op = measured_state_.try_get();
  if (measured_state_op.has_value())
  {
    reset_controller_measured_state_msg(current_state_, reference_and_state_dof_names_);
    measured_state_.try_set(current_state_);
  }

  reference_interfaces_.assign(
    reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  measured_state_values_.assign(
    measured_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  // prefixed save_i_term parameter is read from ROS parameters
  for (auto & pid : pids_)
  {
    pid->reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PidController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref_op = input_ref_.try_get();
  if (current_ref_op.has_value())
  {
    current_ref_ = current_ref_op.value();
  }

  for (size_t i = 0; i < dof_; ++i)
  {
    if (!std::isnan(current_ref_.values[i]))
    {
      reference_interfaces_[i] = current_ref_.values[i];
      const size_t dof_reference_size = dof_ * params_.reference_and_state_interfaces.size();
      if (dof_reference_size == 2 * dof_ && !std::isnan(current_ref_.values_dot[i]))
      {
        reference_interfaces_[dof_ + i] = current_ref_.values_dot[i];
      }
      current_ref_.values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  // save cleared input_ref_
  input_ref_.try_set(current_ref_);
  return controller_interface::return_type::OK;
}

controller_interface::return_type PidController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  param_listener_->try_get_params(params_);

  // Update feedback either from external measured state or from state interfaces
  if (params_.use_external_measured_states)
  {
    auto measured_state_op = measured_state_.try_get();
    if (measured_state_op.has_value())
    {
      current_state_ = measured_state_op.value();
    }
    for (size_t i = 0; i < dof_; ++i)
    {
      measured_state_values_[i] = current_state_.values[i];
      if (measured_state_values_.size() == 2 * dof_)
      {
        measured_state_values_[dof_ + i] = current_state_.values_dot[i];
      }
    }
  }
  else
  {
    for (size_t i = 0; i < measured_state_values_.size(); ++i)
    {
      const auto state_interface_value_op = state_interfaces_[i].get_optional();
      if (!state_interface_value_op.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve the state interface value for %s",
          state_interfaces_[i].get_name().c_str());
        continue;
      }
      measured_state_values_[i] = state_interface_value_op.value();
    }
  }

  // Fill the information of the exported state interfaces
  for (size_t i = 0; i < measured_state_values_.size(); ++i)
  {
    state_interfaces_values_[i] = measured_state_values_[i];
  }

  // Calculate size of DOF references for indexing
  const size_t dof_reference_size = dof_ * params_.reference_and_state_interfaces.size();

  if (params_.export_gain_references)
  {
    size_t gains_start_index = dof_reference_size;
    for (size_t i = 0; i < dof_; ++i)
    {
      auto current_pid_gains = pids_[i]->get_gains();
      for (size_t j = 0; j < GAIN_INTERFACES.size(); ++j)
      {
        const size_t buffer_index = gains_start_index + i + j * dof_;
        const double new_gain_value = reference_interfaces_[buffer_index];
        if (std::isfinite(new_gain_value))
        {
          const size_t gain_type = GAIN_TYPES_INDEX[j];
          switch (gain_type)
          {
            case 0:  // P gain
              current_pid_gains.p_gain_ = new_gain_value;
              pids_[i]->set_gains(current_pid_gains);
              break;
            case 1:  // I gain
              current_pid_gains.i_gain_ = new_gain_value;
              pids_[i]->set_gains(current_pid_gains);
              break;
            case 2:  // D gain
              current_pid_gains.d_gain_ = new_gain_value;
              pids_[i]->set_gains(current_pid_gains);
              break;
          }
          reference_interfaces_[buffer_index] = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
  }

  // Iterate through all the dofs to calculate the output command
  for (size_t i = 0; i < dof_; ++i)
  {
    double tmp_command = 0.0;

    if (std::isfinite(reference_interfaces_[i]) && std::isfinite(measured_state_values_[i]))
    {
      // calculate feed-forward
      if (dof_reference_size == 2 * dof_)
      {
        // two interfaces
        if (std::isfinite(reference_interfaces_[dof_ + i]))
        {
          tmp_command = reference_interfaces_[dof_ + i] *
                        params_.gains.dof_names_map[params_.dof_names[i]].feedforward_gain;
        }
      }
      else  // one interface
      {
        tmp_command = reference_interfaces_[i] *
                      params_.gains.dof_names_map[params_.dof_names[i]].feedforward_gain;
      }

      double error = reference_interfaces_[i] - measured_state_values_[i];
      if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
      {
        // for continuous angles the error is normalized between -pi<error<pi
        error =
          angles::shortest_angular_distance(measured_state_values_[i], reference_interfaces_[i]);
      }

      // checking if there are two interfaces
      if (dof_reference_size == 2 * dof_ && measured_state_values_.size() == 2 * dof_)
      {
        if (
          std::isfinite(reference_interfaces_[dof_ + i]) &&
          std::isfinite(measured_state_values_[dof_ + i]))
        {
          // use calculation with 'error' and 'error_dot'
          tmp_command += pids_[i]->compute_command(
            error, reference_interfaces_[dof_ + i] - measured_state_values_[dof_ + i], period);
        }
        else
        {
          // Fallback to calculation with 'error' only
          tmp_command += pids_[i]->compute_command(error, period);
        }
      }
      else
      {
        // use calculation with 'error' only
        tmp_command += pids_[i]->compute_command(error, period);
      }

      // write calculated values
      auto success = command_interfaces_[i].set_value(tmp_command);
      if (!success)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Failed to set command value for %s",
          command_interfaces_[i].get_name().c_str());
      }
    }
  }

  if (state_publisher_)
  {
    state_msg_.header.stamp = time;
    for (size_t i = 0; i < dof_; ++i)
    {
      state_msg_.dof_states[i].reference = reference_interfaces_[i];
      state_msg_.dof_states[i].feedback = measured_state_values_[i];
      if (dof_reference_size == 2 * dof_ && measured_state_values_.size() == 2 * dof_)
      {
        state_msg_.dof_states[i].feedback_dot = measured_state_values_[dof_ + i];
      }
      state_msg_.dof_states[i].error = reference_interfaces_[i] - measured_state_values_[i];
      if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
      {
        // for continuous angles the error is normalized between -pi<error<pi
        state_msg_.dof_states[i].error =
          angles::shortest_angular_distance(measured_state_values_[i], reference_interfaces_[i]);
      }
      if (dof_reference_size == 2 * dof_ && measured_state_values_.size() == 2 * dof_)
      {
        state_msg_.dof_states[i].error_dot =
          reference_interfaces_[dof_ + i] - measured_state_values_[dof_ + i];
      }
      state_msg_.dof_states[i].time_step = period.seconds();
      // Command can store the old calculated values. This should be obvious because at least one
      // another value is NaN.
      const auto command_interface_value_op = command_interfaces_[i].get_optional();

      if (!command_interface_value_op.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve the command interface value for %s",
          command_interfaces_[i].get_name().c_str());
      }
      else
      {
        state_msg_.dof_states[i].output = command_interface_value_op.value();
      }
    }
    state_publisher_->try_publish(state_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pid_controller::PidController, controller_interface::ChainableControllerInterface)
