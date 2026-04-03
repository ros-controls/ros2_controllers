// Copyright 2024 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#ifndef FORWARD_STATE_CONTROLLER__FORWARD_STATE_CONTROLLER_HPP_
#define FORWARD_STATE_CONTROLLER__FORWARD_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "forward_state_controller/forward_state_controller_parameters.hpp"

namespace forward_state_controller
{
/**
 * \brief Forward state controller that forwards state interfaces to command interfaces.
 *
 *
 * \param state_interfaces List of state interface names to read from.
 * \param forward_state Map from each state interface name to the list of command interfaces
 *   to forward its value to.
 *
 * Configuration example:
 * \code{.yaml}
 *   state_interfaces:
 *     - joint1/position
 *     - joint1/velocity
 *   forward_state:
 *     joint1/position:
 *       to_command: ["joint2/position", "joint3/position"]
 *     joint1/velocity:
 *       to_command: ["joint2/velocity"]
 * \endcode
 */
class ForwardStateController : public controller_interface::ControllerInterface
{
public:
  ForwardStateController();

  ~ForwardStateController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> state_interface_names_;
  std::vector<std::string> command_interface_names_;

  std::unordered_map<std::size_t, std::vector<std::size_t>> state_to_command_map_;
};

}  // namespace forward_state_controller

#endif  // FORWARD_STATE_CONTROLLER__FORWARD_STATE_CONTROLLER_HPP_
