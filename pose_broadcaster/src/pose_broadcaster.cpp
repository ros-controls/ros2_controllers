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
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
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
    params_.interface_names.position.z.empty() && params_.interface_names.orientation.x.empty() &&
    params_.interface_names.orientation.y.empty() &&
    params_.interface_names.orientation.z.empty() && params_.interface_names.orientation.w.empty();

  if (params_.pose_name.empty() && no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'pose_name' or 'interface_names.[position.[x|y|z]|orientation.[x|y|z|w]]' parameter has to "
      "be "
      "specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.pose_name.empty() && !no_interface_names_defined)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'pose_name' and 'interface_names.[position.[x|y|z]|orientation.[x|y|z|w]]' parameters can "
      "not "
      "be specified together.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.pose_name.empty())
  {
    interface_names_ = {
      params_.interface_names.position.x,    params_.interface_names.position.y,
      params_.interface_names.position.z,    params_.interface_names.orientation.x,
      params_.interface_names.orientation.y, params_.interface_names.orientation.z,
      params_.interface_names.orientation.w};
  }
  else
  {
    interface_names_ = {params_.pose_name + "/position.x",    params_.pose_name + "/position.y",
                        params_.pose_name + "/position.z",    params_.pose_name + "/orientation.x",
                        params_.pose_name + "/orientation.y", params_.pose_name + "/orientation.z",
                        params_.pose_name + "/orientation.w"};
  }

  try
  {
    pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/pose", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        pose_publisher_);
  }
  catch (const std::exception & ex)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message: %s\n",
      ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
  realtime_publisher_->unlock();

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
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;

    realtime_publisher_->msg_.pose.position.x = state_interfaces_[0].get_value();
    realtime_publisher_->msg_.pose.position.y = state_interfaces_[1].get_value();
    realtime_publisher_->msg_.pose.position.z = state_interfaces_[2].get_value();

    realtime_publisher_->msg_.pose.orientation.x = state_interfaces_[3].get_value();
    realtime_publisher_->msg_.pose.orientation.y = state_interfaces_[4].get_value();
    realtime_publisher_->msg_.pose.orientation.z = state_interfaces_[5].get_value();
    realtime_publisher_->msg_.pose.orientation.w = state_interfaces_[6].get_value();

    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pose_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pose_broadcaster::PoseBroadcaster, controller_interface::ControllerInterface)
