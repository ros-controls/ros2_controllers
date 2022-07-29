// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef JOINT_STATE_BROADCASTER__JOINT_STATE_BROADCASTER_HPP_
#define JOINT_STATE_BROADCASTER__JOINT_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "joint_state_broadcaster/visibility_control.h"
#include "joint_state_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace joint_state_broadcaster
{
/**
 * \brief Joint State Broadcaster for all or some state in a ros2_control system.
 *
 * JointStateBroadcaster publishes state interfaces from ros2_control as ROS messages.
 * There is a possibility to publish all available states (typical use), or only specific ones.
 * The latter is, for example, used when hardware provides multiple measurement sources for some
 * of its states, e.g., position.
 * It is possible to define a mapping of measurements
 * from different sources stored in custom interfaces to standard dynamic names in JointState
 * message.
 * If "joints" or "interfaces" parameter is empty, all available states are published.
 *
 * \param use_local_topics Flag to publish topics in local namespace.
 * \param joints Names of the joints to publish.
 * \param interfaces Names of interfaces to publish.
 * \param map_interface_to_joint_state.{HW_IF_POSITION|HW_IF_VELOCITY|HW_IF_EFFORT} mapping
 * between custom interface names and standard names in sensor_msgs::msg::JointState message.
 *
 * Publishes to:
 * - \b joint_states (sensor_msgs::msg::JointState): Joint states related to movement
 * (position, velocity, effort).
 * - \b dynamic_joint_states (control_msgs::msg::DynamicJointState): Joint states regardless of
 * its interface type.
 */
class JointStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  JOINT_STATE_BROADCASTER_PUBLIC
  JointStateBroadcaster();

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_STATE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  bool init_joint_data();
  void init_joint_state_msg();
  void init_dynamic_joint_state_msg();
  bool use_all_available_interfaces() const;

protected:
  // Optional parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::unordered_map<std::string, std::string> map_interface_to_joint_state_;

  //  For the JointState message,
  //  we store the name of joints with compatible interfaces
  std::vector<std::string> joint_names_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_joint_state_publisher_;

  //  For the DynamicJointState format, we use a map to buffer values in for easier lookup
  //  This allows to preserve whatever order or names/interfaces were initialized.
  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::DynamicJointState>>
    dynamic_joint_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>
    realtime_dynamic_joint_state_publisher_;
};

}  // namespace joint_state_broadcaster

#endif  // JOINT_STATE_BROADCASTER__JOINT_STATE_BROADCASTER_HPP_
