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
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "joint_state_controller/visibility_control.h"

#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

namespace ros_controllers
{

class JointStateController : public controller_interface::ControllerInterface
{
public:
  JOINT_STATE_CONTROLLER_PUBLIC
  JointStateController();

  JOINT_STATE_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t
  update() override;

  JOINT_STATE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::vector<const hardware_interface::JointStateHandle *> registered_joint_handles_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>>
  joint_state_publisher_;
  sensor_msgs::msg::JointState joint_state_msg_;
};

}  // namespace ros_controllers

#endif  // JOINT_STATE_CONTROLLER__JOINT_STATE_CONTROLLER_HPP_
