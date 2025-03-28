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
#include <cmath>
#include <rclcpp/logging.hpp>

namespace
{

constexpr auto DEFAULT_POSE_TOPIC = "~/pose";
constexpr auto DEFAULT_TF_TOPIC = "/tf";

}  // namespace

namespace pose_broadcaster
{

bool is_pose_valid(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) &&

         std::abs(
           pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
           pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w -
           1.0) <= 10e-3;
}

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
  state_interfaces_config.names = pose_sensor_->get_state_interface_names();

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

  pose_sensor_ = std::make_unique<semantic_components::PoseSensor>(params_.pose_name);

  // TODO: Remove this check and its contents
  if (params_.tf.publish_rate == 0.0)
  {
    tf_publish_period_ = std::nullopt;
  }
  else
  {
    tf_publish_period_ =
      std::optional{rclcpp::Duration::from_seconds(1.0 / params_.tf.publish_rate)};
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[deprecated] tf.publish_rate parameter is deprecated, please set the value to 0.0. "
      "The publish rate of TF messages is no longer limited by this parameter.");
  }

  try
  {
    pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      DEFAULT_POSE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        pose_publisher_);

    if (params_.tf.enable)
    {
      tf_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        DEFAULT_TF_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_tf_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
          tf_publisher_);
    }
  }
  catch (const std::exception & ex)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message: %s\n",
      ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize pose message
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
  realtime_publisher_->unlock();

  // Initialize tf message if tf publishing is enabled
  if (realtime_tf_publisher_)
  {
    realtime_tf_publisher_->lock();

    realtime_tf_publisher_->msg_.transforms.resize(1);
    auto & tf_transform = realtime_tf_publisher_->msg_.transforms.front();
    tf_transform.header.frame_id = params_.frame_id;
    if (params_.tf.child_frame_id.empty())
    {
      tf_transform.child_frame_id = params_.pose_name;
    }
    else
    {
      tf_transform.child_frame_id = params_.tf.child_frame_id;
    }

    realtime_tf_publisher_->unlock();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pose_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pose_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  geometry_msgs::msg::Pose pose;
  pose_sensor_->get_values_as_message(pose);

  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.pose = pose;
    realtime_publisher_->unlockAndPublish();
  }
  if (!is_pose_valid(pose))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Invalid pose [%f, %f, %f], [%f, %f, %f, %f]", pose.position.x, pose.position.y,
      pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w);
  }
  else if (realtime_tf_publisher_ && realtime_tf_publisher_->trylock())
  {
    auto & tf_transform = realtime_tf_publisher_->msg_.transforms[0];
    tf_transform.header.stamp = time;

    tf_transform.transform.translation.x = pose.position.x;
    tf_transform.transform.translation.y = pose.position.y;
    tf_transform.transform.translation.z = pose.position.z;

    tf_transform.transform.rotation.x = pose.orientation.x;
    tf_transform.transform.rotation.y = pose.orientation.y;
    tf_transform.transform.rotation.z = pose.orientation.z;
    tf_transform.transform.rotation.w = pose.orientation.w;

    realtime_tf_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pose_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pose_broadcaster::PoseBroadcaster, controller_interface::ControllerInterface)
