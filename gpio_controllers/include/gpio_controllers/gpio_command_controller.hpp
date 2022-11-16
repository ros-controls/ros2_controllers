// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef GPIO_CONTROLLERS__GPIO_COMMAND_CONTROLLER_HPP_
#define GPIO_CONTROLLERS__GPIO_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "gpio_controllers/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace gpio_controllers
{
using CmdType = std_msgs::msg::Float64MultiArray;
using StateType = control_msgs::msg::DynamicJointState;
using CallbackReturn = controller_interface::CallbackReturn;

class GpioCommandController : public controller_interface::ControllerInterface
{
public:
  GPIO_COMMAND_CONTROLLER_PUBLIC
  GpioCommandController();

  GPIO_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  GPIO_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> gpio_names_;
  std::unordered_map<std::string, std::vector<std::string>> interface_names_;
  std::vector<std::string> interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr gpios_command_subscriber_;

  std::shared_ptr<rclcpp::Publisher<StateType>> gpio_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_gpio_state_publisher_;

  std::string logger_name_;
};

}  // namespace gpio_controllers

#endif  // GPIO_CONTROLLERS__GPIO_COMMAND_CONTROLLER_HPP_
