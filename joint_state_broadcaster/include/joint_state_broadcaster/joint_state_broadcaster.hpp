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

#ifndef JOINT_STATE_CONTROLLER__JOINT_STATE_CONTROLLER_HPP_
#define JOINT_STATE_CONTROLLER__JOINT_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "joint_state_controller/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace joint_state_controller
{

class JointStateController : public controller_interface::ControllerInterface
{
public:
  JOINT_STATE_CONTROLLER_PUBLIC
  JointStateController();

  JOINT_STATE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  JOINT_STATE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JOINT_STATE_CONTROLLER_PUBLIC
  controller_interface::return_type
  update() override;

  JOINT_STATE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  JOINT_STATE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  JOINT_STATE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  bool init_joint_data();
  void init_joint_state_msg();
  void init_dynamic_joint_state_msg();

protected:
  //  For the JointState message,
  //  we store the name of joints with compatible interfaces
  std::vector<std::string> joint_names_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
  joint_state_publisher_;
  sensor_msgs::msg::JointState joint_state_msg_;

  //  For the DynamicJointState format, we use a map to buffer values in for easier lookup
  //  This allows to preserve whatever order or names/interfaces were initialized.
  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::DynamicJointState>>
  dynamic_joint_state_publisher_;
  control_msgs::msg::DynamicJointState dynamic_joint_state_msg_;
};

}  // namespace joint_state_controller

#endif  // JOINT_STATE_CONTROLLER__JOINT_STATE_CONTROLLER_HPP_
