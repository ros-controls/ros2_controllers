// Copyright (c) 2025, b-robotized
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
#define VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <vda5050_safety_state_broadcaster/vda5050_safety_state_broadcaster_parameters.hpp>
#include "control_msgs/msg/vda5050_safety_state.hpp"

namespace vda5050_safety_state_broadcaster
{

/**
 * \brief VDA5050 safety state broadcaster for all or some state in a ros2_control system.
 *
 * Vda5050SafetyStateBroadcaster publishes state interfaces from ros2_control as ROS messages.
 * The state interfaces published can be configured via parameters:
 *
 * \param fieldViolation_interfaces that are used to acknowledge field violation events by setting
 * the interface to 1.0.
 * \param eStop_manual_interfaces that are used to manually acknowledge eStop events by setting the
 * interface to 1.0.
 * \param eStop_remote_interfaces that are used to remotely acknowledge eStop events by setting the
 * interface to 1.0.
 * \param eStop_autoack_interfaces that are used to autoacknowledge eStop events by setting the
 * interface to 1.0.
 *
 * Publishes to:
 *
 * - \b vda5050_safety_state (control_msgs::msg::VDA5050SafetyState): safety state of the combined
 * safety interfaces according the priority: eStop_manual > eStop_remote > eStop_autoack.
 *
 */
class Vda5050SafetyStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  Vda5050SafetyStateBroadcaster();

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
  vda5050_safety_state_broadcaster::Params params_;

  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::VDA5050SafetyState>>
    realtime_vda5050_safety_state_publisher_;

private:
  std::shared_ptr<vda5050_safety_state_broadcaster::ParamListener> param_listener_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::VDA5050SafetyState>>
    vda5050_safety_state_publisher_;

  /**
   * @brief Determines the current E-stop state based on the state interfaces.
   * @return The E-stop type as defined in control_msgs::msg::VDA5050SafetyState.
   */
  control_msgs::msg::VDA5050SafetyState::_e_stop_type determineEstopState();

  struct InterfaceIds
  {
    int manual_start = 0;
    int remote_start = 0;
    int autoack_start = 0;
    int total_interfaces = 0;
  };

  InterfaceIds itfs_ids_;
  bool fieldViolation_value = false;
  std::string estop_msg = control_msgs::msg::VDA5050SafetyState::NONE;

  /**
   * @brief Safely converts a double value to bool, treating NaN as false.
   * @param value The double value to convert.
   * @return true if value is not NaN and not zero, false otherwise.
   */
  bool safe_double_to_bool(double value) const
  {
    if (std::isnan(value))
    {
      return false;
    }
    return value != 0.0;
  }
};

}  // namespace vda5050_safety_state_broadcaster

#endif  // VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
