// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "pid_controller/pid_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/single_dof_state.hpp"
#include "controller_interface/helpers.hpp"

// TODO(destogl): should we add some knowledge from the control there here?
// The main knowledge could be with regard to the input and output interfaces and their phyical
// meaning, for example:
//
// input POSITION; output VELOCITY --> PI CONTROLLER
// input VELOCITY; output ACCELERATION --> PI CONTROLLER
//
// input VELOCITY; output POSITION --> PD CONTROLLER
// input ACCELERATION; output VELOCITY --> PD CONTROLLER
//
// input POSITION; output POSITION --> PID CONTROLLER
// input VELOCITY; output VELOCITY --> PID CONTROLLER
// input ACCELERATION; output ACCELERATION --> PID CONTROLLER
// input EFFORT; output EFFORT --> PID CONTROLLER
//

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerCommandMsg = pid_controller::PidController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  reset_controller_reference_msg(msg, dof_names);
}

}  // namespace

namespace pid_controller
{
PidController::PidController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
  control_mode_.initRT(feedforward_mode_type::OFF);

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
    pids_[i] =
      std::make_shared<control_toolbox::PidROS>(get_node(), "gains." + params_.dof_names[i]);
    pids_[i]->initPid();
  }
}

controller_interface::CallbackReturn PidController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  configure_parameters();

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&PidController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.dof_names);
  input_ref_.writeFromNonRT(msg);

  // input state Subscriber and callback
  if (params_.use_external_measured_states)
  {
    auto measured_state_callback =
      [&](const std::shared_ptr<ControllerMeasuredStateMsg> msg) -> void {
      measured_state_.writeFromNonRT(msg);
    };
    measured_state_subscriber_ = get_node()->create_subscription<ControllerMeasuredStateMsg>(
      "~/measured_state", subscribers_qos, measured_state_callback);
  }
  std::shared_ptr<ControllerMeasuredStateMsg> measured_state_msg =
    std::make_shared<ControllerMeasuredStateMsg>();
  reset_controller_measured_state_msg(measured_state_msg, reference_and_state_dof_names_);
  measured_state_.writeFromNonRT(measured_state_msg);

  auto set_feedforward_control_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response) {
      if (request->data)
      {
        control_mode_.writeFromNonRT(feedforward_mode_type::ON);
      }
      else
      {
        control_mode_.writeFromNonRT(feedforward_mode_type::OFF);
      }
      response->success = true;
    };

  set_feedforward_control_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_feedforward_control", set_feedforward_control_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
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
  state_publisher_->lock();
  state_publisher_->msg_.dof_states.reserve(reference_and_state_dof_names_.size());
  for (size_t i = 0; i < reference_and_state_dof_names_.size(); ++i)
  {
    state_publisher_->msg_.dof_states[i] = control_msgs::msg::SingleDOFState();
    state_publisher_->msg_.dof_states[i].name = reference_and_state_dof_names_[i];
  }
  state_publisher_->unlock();

  // TODO(destogl): Add separate timer-callback for the controller status publisher

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
    input_ref_.writeFromNonRT(ref_msg);
  }
  else if (
    msg->dof_names.size() == reference_and_state_dof_names_.size() &&
    msg->values.size() == reference_and_state_dof_names_.size())
  {
    auto ref_msg = msg;  // simple initialization

    // sort values in the ref_msg
    ref_msg->dof_names = reference_and_state_dof_names_;
    ref_msg->values.assign(ref_msg->values.size(), std::numeric_limits<double>::quiet_NaN());

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

      auto position = std::distance(ref_msg->dof_names.begin(), found_it);
      ref_msg->values[position] = msg->values[i];
    }

    if (all_found)
    {
      input_ref_.writeFromNonRT(ref_msg);
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
  for (const auto & joint : params_.dof_names)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.command_interface);
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
      for (const auto & joint : reference_and_state_dof_names_)
      {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> PidController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(
    dof_ * params_.reference_and_state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  size_t index = 0;
  for (const auto & interface : params_.reference_and_state_interfaces)
  {
    for (const auto & joint : reference_and_state_dof_names_)
    {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), joint + "/" + interface, &reference_interfaces_[index]));
      ++index;
    }
  }

  return reference_interfaces;
}

bool PidController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn PidController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_controller_reference_msg(*(input_ref_.readFromRT()), reference_and_state_dof_names_);
  reset_controller_measured_state_msg(
    *(measured_state_.readFromRT()), reference_and_state_dof_names_);

  for (size_t i = 0; i < dof_; ++i)
  {
    reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // TODO(destogl): make here parameter update

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < dof_; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PidController::update_reference_from_subscribers()
{
  auto current_ref = input_ref_.readFromRT();

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->values[i]))
    {
      reference_interfaces_[i] = (*current_ref)->values[i];

      (*current_ref)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type PidController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(destogl): make here runtime parameter update if allowed

  auto measured_state = measured_state_.readFromRT();

  for (size_t i = 0; i < dof_; ++i)
  {
    double tmp_command = std::numeric_limits<double>::quiet_NaN();

    if (
      *(control_mode_.readFromRT()) == feedforward_mode_type::ON &&
      !std::isnan(reference_interfaces_[i]) && !std::isnan((*measured_state)->values[i]))
    {
      // calculate feed-forward
      if (!std::isnan(reference_interfaces_[dof_ + i]))
      {
        tmp_command = reference_interfaces_[dof_ + i] *
                      params_.gains.dof_names_map[params_.dof_names[i]].feedforward_gain;
      }
      else
      {
        tmp_command = 0.0;
      }

      if (
        reference_interfaces_.size() > dof_ &&
        (*measured_state)->values.size())  // TODO(denis): make this Always sufficiently big
      {
        if (
          !std::isnan(reference_interfaces_[dof_ + i]) &&
          !std::isnan((*measured_state)->values[dof_ + i]))
        {
          // use calculation with 'error' and 'error_dot'
          tmp_command += pids_[i]->computeCommand(
            reference_interfaces_[i] - (*measured_state)->values[i],
            reference_interfaces_[dof_ + i] - (*measured_state)->values[dof_ + i], period);
        }
        else
        {
          // Fallback to calculation with 'error' only
          tmp_command += pids_[i]->computeCommand(
            reference_interfaces_[i] - (*measured_state)->values[i], period);
        }
      }
      else
      {
        // use calculation with 'error' only
        tmp_command +=
          pids_[i]->computeCommand(reference_interfaces_[i] - (*measured_state)->values[i], period);
      }
    }

    command_interfaces_[i].set_value(tmp_command);
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    for (size_t i = 0; i < dof_; ++i)
    {
      state_publisher_->msg_.dof_states[i].set_point = reference_interfaces_[i];
      state_publisher_->msg_.dof_states[i].process_value = (*measured_state)->values[i];
      state_publisher_->msg_.dof_states[i].error =
        reference_interfaces_[i] - (*measured_state)->values[i];
      state_publisher_->msg_.dof_states[i].time_step = period.nanoseconds();
      state_publisher_->msg_.dof_states[i].command = command_interfaces_[i].get_value();

      auto gains = pids_[i]->getGains();
      state_publisher_->msg_.dof_states[i].p = gains.p_gain_;
      state_publisher_->msg_.dof_states[i].i = gains.i_gain_;
      state_publisher_->msg_.dof_states[i].d = gains.d_gain_;
      state_publisher_->msg_.dof_states[i].i_clamp = gains.i_max_;
      state_publisher_->msg_.dof_states[i].antiwindup = gains.antiwindup_;
    }
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pid_controller::PidController, controller_interface::ChainableControllerInterface)
