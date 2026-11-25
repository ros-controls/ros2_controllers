// Copyright 2021, PickNik Inc.
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

#ifndef CARTESIAN_CONTROLLERS__TWIST_CONTROLLER_HPP_
#define CARTESIAN_CONTROLLERS__TWIST_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "cartesian_controllers/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace cartesian_controllers
{
using CmdType = geometry_msgs::msg::TwistStamped;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TwistController : public controller_interface::ControllerInterface
{
public:
  CARTESIAN_CONTROLLERS_PUBLIC
  TwistController();

  CARTESIAN_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CARTESIAN_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CARTESIAN_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  CARTESIAN_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string joint_name_;
  std::vector<std::string> interface_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr twist_command_subscriber_;

  std::string logger_name_;
};

}  // namespace cartesian_controllers

#endif  // CARTESIAN_CONTROLLERS__TWIST_CONTROLLER_HPP_
