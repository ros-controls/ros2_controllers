// Copyright 2024 FZI Forschungszentrum Informatik
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
#include "pose_broadcaster/pose_broadcaster.hpp"

namespace pose_broadcaster
{

controller_interface::InterfaceConfiguration PoseBroadcaster::command_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{};
}

controller_interface::InterfaceConfiguration PoseBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::copy(
    interface_names_.cbegin(), interface_names_.cend(),
    std::back_inserter(state_interfaces_config.names));

  return state_interfaces_config;
}

controller_interface::CallbackReturn PoseBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & ex)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  const bool no_interface_names_defined =
    params_.interface_names.position.x.empty() && params_.interface_names.position.y.empty() &&
    params_.interface_names.position.z.empty() && params_.interface_names.orientation.r.empty() &&
    params_.interface_names.orientation.p.empty() && params_.interface_names.orientation.y.empty();

  if (params_.pose_name.empty() && no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'pose_name' or 'interface_names.[position.[x|y|z]|orientation.[r|p|y]]' parameter has to be "
      "specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.pose_name.empty() && !no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'pose_name' and 'interface_names.[position.[x|y|z]|orientation.[r|p|y]]' parameters can not "
      "be specified together.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.pose_name.empty())
  {
    interface_names_ = {
      params_.interface_names.position.x,    params_.interface_names.position.y,
      params_.interface_names.position.z,    params_.interface_names.orientation.r,
      params_.interface_names.orientation.p, params_.interface_names.orientation.y};
  }
  else
  {
    interface_names_ = {params_.pose_name + "/position.x",    params_.pose_name + "/position.y",
                        params_.pose_name + "/position.z",    params_.pose_name + "/orientation.r",
                        params_.pose_name + "/orientation.p", params_.pose_name + "/orientation.y"};
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

}  // namespace pose_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pose_broadcaster::PoseBroadcaster, controller_interface::ControllerInterface)
