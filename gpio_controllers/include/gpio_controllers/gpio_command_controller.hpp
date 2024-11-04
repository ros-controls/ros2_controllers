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
#include "gpio_command_controller_parameters.hpp"
#include "gpio_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "urdf/model.hpp"

namespace gpio_controllers
{
using CmdType = control_msgs::msg::DynamicJointState;
using StateType = control_msgs::msg::DynamicJointState;
using CallbackReturn = controller_interface::CallbackReturn;
using InterfacesNames = std::vector<std::string>;
using MapOfReferencesToCommandInterfaces = std::unordered_map<
  std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using MapOfReferencesToStateInterfaces =
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using StateInterfaces =
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

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

private:
  void store_command_interface_types();
  void store_state_interface_types();
  void initialize_gpio_state_msg();
  void update_gpios_states();
  controller_interface::return_type update_gpios_commands();
  template <typename T>
  std::unordered_map<std::string, std::reference_wrapper<T>> create_map_of_references_to_interfaces(
    const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces);
  template <typename T>
  bool check_if_configured_interfaces_matches_received(
    const InterfacesNames & interfaces_from_params, const T & configured_interfaces);
  void apply_state_value(
    StateType & state_msg, std::size_t gpio_index, std::size_t interface_index) const;
  void apply_command(
    const CmdType & gpio_commands, std::size_t gpio_index,
    std::size_t command_interface_index) const;
  bool should_broadcast_all_interfaces_of_configured_gpios() const;
  void set_all_state_interfaces_of_configured_gpios();
  InterfacesNames get_gpios_state_interfaces_names(const std::string & gpio_name) const;
  bool update_dynamic_map_parameters();
  std::vector<hardware_interface::ComponentInfo> get_gpios_from_urdf() const;

protected:
  InterfacesNames command_interface_types_;
  InterfacesNames state_interface_types_;
  MapOfReferencesToCommandInterfaces command_interfaces_map_;
  MapOfReferencesToStateInterfaces state_interfaces_map_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_{};
  rclcpp::Subscription<CmdType>::SharedPtr gpios_command_subscriber_{};

  std::shared_ptr<rclcpp::Publisher<StateType>> gpio_state_publisher_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_gpio_state_publisher_{};

  std::shared_ptr<gpio_command_controller_parameters::ParamListener> param_listener_{};
  gpio_command_controller_parameters::Params params_;
  urdf::Model model_;
};

}  // namespace gpio_controllers

#endif  // GPIO_CONTROLLERS__GPIO_COMMAND_CONTROLLER_HPP_
